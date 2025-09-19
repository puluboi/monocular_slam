#include "camera.hpp"
#include <iostream>
#include <sys/mman.h>
#include <chrono>

PiCamera::PiCamera() 
    : is_initialized_(false), is_capturing_(false),
      width_(640), height_(480), framerate_(60) {
}

PiCamera::~PiCamera() {
    release();
}

bool PiCamera::initialize() {
    if (is_initialized_) {
        return true;
    }
    
    std::cout << "Initializing libcamera..." << std::endl;
    
    // Create camera manager
    camera_manager_ = std::make_unique<libcamera::CameraManager>();
    
    // Start camera manager
    int ret = camera_manager_->start();
    if (ret) {
        std::cerr << "Failed to start camera manager" << std::endl;
        return false;
    }
    
    // Get available cameras
    auto cameras = camera_manager_->cameras();
    if (cameras.empty()) {
        std::cerr << "No cameras available" << std::endl;
        return false;
    }
    
    // Use the first available camera
    camera_ = cameras[0];
    std::cout << "Using camera: " << camera_->id() << std::endl;
    
    // Acquire the camera
    ret = camera_->acquire();
    if (ret) {
        std::cerr << "Failed to acquire camera" << std::endl;
        return false;
    }
    
    is_initialized_ = true;
    std::cout << "Camera initialized successfully!" << std::endl;
    return true;
}

bool PiCamera::startCapture() {
    if (!is_initialized_) {
        std::cerr << "Camera not initialized" << std::endl;
        return false;
    }
    
    if (is_capturing_) {
        return true;
    }
    
    std::cout << "Starting camera capture..." << std::endl;
    
    // Generate camera configuration
    config_ = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!config_) {
        std::cerr << "Failed to generate camera configuration" << std::endl;
        return false;
    }
    
    // Configure the stream
    libcamera::StreamConfiguration &streamConfig = config_->at(0);
    streamConfig.size.width = width_;
    streamConfig.size.height = height_;
    // Try YUYV first as it's more commonly supported
    streamConfig.pixelFormat = libcamera::formats::YUYV;
    
    // Validate configuration
    libcamera::CameraConfiguration::Status validation = config_->validate();
    if (validation == libcamera::CameraConfiguration::Invalid) {
        std::cerr << "Invalid camera configuration" << std::endl;
        return false;
    }
    
    if (validation == libcamera::CameraConfiguration::Adjusted) {
        std::cout << "Camera configuration adjusted" << std::endl;
        std::cout << "Final format: " << streamConfig.pixelFormat.toString() << std::endl;
        std::cout << "Final size: " << streamConfig.size.width << "x" << streamConfig.size.height << std::endl;
    }
    
    // Configure the camera
    int ret = camera_->configure(config_.get());
    if (ret) {
        std::cerr << "Failed to configure camera" << std::endl;
        return false;
    }
    
    // Create frame buffer allocator
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    
    // Allocate buffers for the stream
    libcamera::Stream *stream = streamConfig.stream();
    ret = allocator_->allocate(stream);
    if (ret < 0) {
        std::cerr << "Failed to allocate buffers" << std::endl;
        return false;
    }
    
    std::cout << "Allocated " << allocator_->buffers(stream).size() << " buffers" << std::endl;
    
    // Create requests
    const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = allocator_->buffers(stream);
    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request) {
            std::cerr << "Failed to create request" << std::endl;
            return false;
        }
        
        const std::unique_ptr<libcamera::FrameBuffer> &buffer = buffers[i];
        ret = request->addBuffer(stream, buffer.get());
        if (ret < 0) {
            std::cerr << "Failed to add buffer to request" << std::endl;
            return false;
        }
        
        requests_.push_back(std::move(request));
    }
    
    // Connect the requestCompleted signal
    camera_->requestCompleted.connect(this, &PiCamera::requestComplete);
    
    // Start the camera
    ret = camera_->start();
    if (ret) {
        std::cerr << "Failed to start camera" << std::endl;
        return false;
    }
    
    // Queue initial requests
    for (std::unique_ptr<libcamera::Request> &request : requests_) {
        ret = camera_->queueRequest(request.get());
        if (ret < 0) {
            std::cerr << "Failed to queue request" << std::endl;
            return false;
        }
    }
    
    is_capturing_ = true;
    std::cout << "Camera capture started!" << std::endl;
    return true;
}

cv::Mat PiCamera::captureFrame() {
    if (!is_capturing_) {
        return cv::Mat();
    }
    
    std::unique_lock<std::mutex> lock(frame_mutex_);
    
    // Wait for a frame with timeout
    if (frame_condition_.wait_for(lock, std::chrono::milliseconds(1000), 
                                  [this] { return !frame_queue_.empty(); })) {
        cv::Mat frame = frame_queue_.front();
        frame_queue_.pop();
        return frame;
    }
    
    // Timeout - return empty frame
    return cv::Mat();
}

void PiCamera::stopCapture() {
    if (!is_capturing_) {
        return;
    }
    
    std::cout << "Stopping camera capture..." << std::endl;
    
    camera_->stop();
    camera_->requestCompleted.disconnect(this, &PiCamera::requestComplete);
    
    requests_.clear();
    allocator_.reset();
    
    // Clear frame queue
    std::lock_guard<std::mutex> lock(frame_mutex_);
    while (!frame_queue_.empty()) {
        frame_queue_.pop();
    }
    
    is_capturing_ = false;
    std::cout << "Camera capture stopped" << std::endl;
}

void PiCamera::release() {
    stopCapture();
    
    if (camera_) {
        camera_->release();
        camera_.reset();
    }
    
    if (camera_manager_) {
        camera_manager_->stop();
        camera_manager_.reset();
    }
    
    is_initialized_ = false;
    std::cout << "Camera released" << std::endl;
}

bool PiCamera::setResolution(int width, int height) {
    if (width > 0 && height > 0 && width <= 4096 && height <= 3072) {
        width_ = width;
        height_ = height;
        return true;
    }
    return false;
}

bool PiCamera::setFramerate(int fps) {
    if (fps > 0 && fps <= 120) {
        framerate_ = fps;
        return true;
    }
    return false;
}

void PiCamera::requestComplete(libcamera::Request *request) {
    if (!request || request->status() == libcamera::Request::RequestCancelled) {
        return;
    }
    
    if (!is_capturing_) {
        return; // Camera is stopping, don't process more frames
    }
    
    try {
        // Get the buffer from the request
        const std::map<const libcamera::Stream *, libcamera::FrameBuffer *> &buffers = request->buffers();
        for (auto bufferPair : buffers) {
            libcamera::FrameBuffer *buffer = bufferPair.second;
            const libcamera::Stream *stream = bufferPair.first;
            
            if (!buffer || !stream) {
                continue;
            }
            
            // Convert buffer to OpenCV Mat
            cv::Mat frame = bufferToMat(buffer, stream->configuration());
            
            if (!frame.empty()) {
                // Add frame to queue (keep only latest frames)
                std::lock_guard<std::mutex> lock(frame_mutex_);
                if (frame_queue_.size() >= 3) {
                    frame_queue_.pop(); // Remove oldest frame
                }
                frame_queue_.push(frame);
                frame_condition_.notify_one();
            }
        }
        
        // Requeue the request only if still capturing
        if (is_capturing_ && camera_) {
            request->reuse(libcamera::Request::ReuseBuffers);
            camera_->queueRequest(request);
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in requestComplete: " << e.what() << std::endl;
    }
}

cv::Mat PiCamera::bufferToMat(libcamera::FrameBuffer *buffer, const libcamera::StreamConfiguration &cfg) {
    if (!buffer || buffer->planes().empty()) {
        std::cerr << "Invalid buffer" << std::endl;
        return cv::Mat();
    }
    
    // Map the buffer
    const libcamera::FrameBuffer::Plane &plane = buffer->planes()[0];
    void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    
    if (memory == MAP_FAILED) {
        std::cerr << "Failed to mmap buffer" << std::endl;
        return cv::Mat();
    }
    
    try {
        cv::Mat result;
        
        // Handle different pixel formats
        if (cfg.pixelFormat == libcamera::formats::YUV420) {
            // Use actual buffer size for YUV420
            size_t y_size = cfg.size.width * cfg.size.height;
            if (plane.length >= y_size) {
                // Create YUV Mat with the actual buffer layout
                cv::Mat yuv(cfg.size.height * 3 / 2, cfg.size.width, CV_8UC1, memory);
                cv::cvtColor(yuv, result, cv::COLOR_YUV2BGR_I420);
            }
        } else if (cfg.pixelFormat == libcamera::formats::YUYV) {
            // YUYV format (packed YUV 4:2:2)
            cv::Mat yuyv(cfg.size.height, cfg.size.width, CV_8UC2, memory);
            cv::cvtColor(yuyv, result, cv::COLOR_YUV2BGR_YUYV);
        } else if (cfg.pixelFormat == libcamera::formats::RGB888) {
            // RGB format
            cv::Mat rgb(cfg.size.height, cfg.size.width, CV_8UC3, memory);
            cv::cvtColor(rgb, result, cv::COLOR_RGB2BGR);
        } else if (cfg.pixelFormat == libcamera::formats::BGR888) {
            // BGR format (direct copy)
            cv::Mat bgr(cfg.size.height, cfg.size.width, CV_8UC3, memory);
            result = bgr.clone();
        } else {
            // Try to handle as raw single channel and convert
            std::cout << "Unknown format, trying raw conversion. Buffer size: " << plane.length 
                     << ", Expected: " << cfg.size.width * cfg.size.height << std::endl;
            
            // Calculate bytes per pixel from buffer size
            size_t total_pixels = cfg.size.width * cfg.size.height;
            size_t bytes_per_pixel = plane.length / total_pixels;
            
            if (bytes_per_pixel == 1) {
                // Single channel (grayscale)
                cv::Mat gray(cfg.size.height, cfg.size.width, CV_8UC1, memory);
                cv::cvtColor(gray, result, cv::COLOR_GRAY2BGR);
            } else if (bytes_per_pixel == 2) {
                // Assume YUYV or similar
                cv::Mat yuyv(cfg.size.height, cfg.size.width, CV_8UC2, memory);
                cv::cvtColor(yuyv, result, cv::COLOR_YUV2BGR_YUYV);
            } else if (bytes_per_pixel == 3) {
                // Assume RGB
                cv::Mat rgb(cfg.size.height, cfg.size.width, CV_8UC3, memory);
                cv::cvtColor(rgb, result, cv::COLOR_RGB2BGR);
            }
        }
        
        // Unmap the buffer
        munmap(memory, plane.length);
        
        return result;
    } catch (const std::exception& e) {
        std::cerr << "Exception in bufferToMat: " << e.what() << std::endl;
        munmap(memory, plane.length);
        return cv::Mat();
    }
}