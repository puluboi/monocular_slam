#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <libcamera/libcamera/libcamera.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

/**
 * @brief PiCamera class for Raspberry Pi camera operations using libcamera
 * 
 * This class provides a proper interface for capturing video frames from
 * the Raspberry Pi camera using the libcamera C++ API directly.
 */
class PiCamera {
private:
    std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    
    std::queue<cv::Mat> frame_queue_;
    std::mutex frame_mutex_;
    std::condition_variable frame_condition_;
    
    bool is_initialized_;
    bool is_capturing_;
    int width_;
    int height_;
    int framerate_;

public:
    /**
     * @brief Constructor - initializes camera with default settings
     */
    PiCamera();
    
    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~PiCamera();
    
    /**
     * @brief Initialize the camera system
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Start video capture
     * @return true if successful, false otherwise
     */
    bool startCapture();
    
    /**
     * @brief Capture a single frame
     * @return OpenCV Mat containing the frame, empty if failed
     */
    cv::Mat captureFrame();
    
    /**
     * @brief Stop video capture
     */
    void stopCapture();
    
    /**
     * @brief Release camera resources
     */
    void release();
    
    /**
     * @brief Set camera resolution (takes effect on next startCapture)
     * @param width Frame width
     * @param height Frame height
     * @return true if parameters are valid
     */
    bool setResolution(int width, int height);

    /// @brief Return's the current camera resolution
    /// @return camera resolution as an int pair
    std::pair<int, int> getResolution() const;
    /**
     * @brief Set camera framerate (takes effect on next startCapture)
     * @param fps Frames per second
     * @return true if parameter is valid
     */
    bool setFramerate(int fps);

private:
    void requestComplete(libcamera::Request *request);
    cv::Mat bufferToMat(libcamera::FrameBuffer *buffer, const libcamera::StreamConfiguration &cfg);
};

#endif // CAMERA_HPP
