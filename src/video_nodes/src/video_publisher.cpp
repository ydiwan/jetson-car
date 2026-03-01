#include <cv_bridge/cv_bridge.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <thread>

namespace video_pub_ns {
// read ros names
using img_msg = sensor_msgs::msg::Image;
using img_pub = rclcpp::Publisher<img_msg>::SharedPtr;
using img_ptr = sensor_msgs::msg::Image::SharedPtr;

// adding ros names
using rclcpp::Node;
using ros_timer = rclcpp::TimerBase::SharedPtr;
using cv_bridge::CvImage;
using rclcpp::init;
using rclcpp::shutdown;
using rclcpp::spin;
using std_msgs::msg::Header;

// adding opencv names
using cv::Mat;
using cv::VideoCapture;

// adding std name
using std::make_shared;
using std::ostringstream;
using std::string;
using std::chrono::milliseconds;

}  // namespace video_pub_ns
using namespace video_pub_ns;

class VideoPublisher : public Node {
   public:
    VideoPublisher() : Node("video_publisher"), should_exit_(false) {
        // Declare parameters
        this->declare_parameter("camera_index", 0);  // index of the camera
        this->declare_parameter("width", 640);       // width of the cam feed
        this->declare_parameter("height", 480);      // height of the cam feed
        this->declare_parameter("fps", 60);          // fps of the cam feed

        // Get parameters
        int cam_index = this->get_parameter("camera_index").as_int();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();

        // Create the working pipeline
        string pipeline = create_pipeline(cam_index, width, height, fps);

        // Open camera
        cap_ = VideoCapture(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera with pipeline: %s",
                         pipeline.c_str());
            return;
        }

        // Create publisher for raw images only
        image_pub_ = this->create_publisher<img_msg>("camera/image_raw", 1);

        // Start capture thread
        capture_thread_ = std::thread(&VideoPublisher::capture_loop, this);

        // Create timer for publishing
        int period_ms = 1000 / fps;
        timer_ = this->create_wall_timer(milliseconds(period_ms),
                                         [this] { publish_frame(); });

        RCLCPP_INFO(this->get_logger(), "Video publisher started at %d fps", fps);
        RCLCPP_INFO(this->get_logger(), "Publishing on: camera/image_raw");
    }

    ~VideoPublisher() {
        should_exit_ = true;
        frame_ready_.notify_all();

        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }

        if (cap_.isOpened()) {
            cap_.release();
        }

        RCLCPP_INFO(this->get_logger(), "Video publisher shutdown complete");
    }

   private:
    VideoCapture cap_;
    img_pub image_pub_;
    ros_timer timer_;

    // Thread-safe frame buffer
    std::thread capture_thread_;
    std::atomic<bool> should_exit_;
    std::mutex frame_mutex_;
    std::condition_variable frame_ready_;
    Mat latest_frame_;
    bool has_new_frame_ = false;

    string create_pipeline(int cam_index, int width, int height, int fps) {
        ostringstream pipeline;

        pipeline << "nvarguscamerasrc sensor-id=" << cam_index
                 << " ! video/x-raw(memory:NVMM),width=(int)" << width << ",height=(int)"
                 << height << ",framerate=(fraction)" << fps << "/1,format=NV12 "
                 << "! nvvidconv "
                 << "! video/x-raw,width=(int)" << width << ",height=(int)" << height
                 << ",format=(string)BGRx "
                 << "! appsink sync=false";

        RCLCPP_INFO(this->get_logger(), "Pipeline: %s", pipeline.str().c_str());
        return pipeline.str();
    }

    void capture_loop() {
        Mat frame;
        while (!should_exit_) {
            if (cap_.read(frame) && !frame.empty()) {
                // Ensure the frame is continuous in memory
                Mat continuous_frame;
                if (!frame.isContinuous()) {
                    continuous_frame = frame.clone();
                } else {
                    continuous_frame = frame;
                }

                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = continuous_frame.clone();
                has_new_frame_ = true;
                frame_ready_.notify_one();
            } else {
                // Small delay to prevent busy-waiting on read failures
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    void publish_frame() {
        Mat frame;
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            if (!has_new_frame_) {
                return;  // No new frame available
            }
            frame = latest_frame_.clone();
            has_new_frame_ = false;
        }

        // Convert BGRx to BGR if necessary
        if (frame.channels() == 4) {
            Mat bgr_frame;
            cv::cvtColor(frame, bgr_frame, cv::COLOR_BGRA2BGR);
            frame = bgr_frame;
        }

        // Ensure proper step/stride
        if (!frame.isContinuous()) {
            frame = frame.clone();  // This creates a continuous copy
        }

        // Create and publish raw image message
        img_ptr msg = CvImage(Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "camera_frame";

        // Debug info (uncomment if needed)
        // RCLCPP_INFO_ONCE(this->get_logger(),
        //     "Image info - Width: %d, Height: %d, Step: %d, Expected step:
        //     %d", frame.cols, frame.rows, frame.step, frame.cols *
        //     frame.channels());

        image_pub_->publish(*msg);
    }
};

int main(int argc, char **argv) {
    init(argc, argv);

    // Use shared_ptr with custom deleter to ensure proper cleanup
    auto node = make_shared<VideoPublisher>();

    // Set up custom signal handler for cleaner shutdown
    rclcpp::on_shutdown(
        [node]() { RCLCPP_INFO(node->get_logger(), "Shutdown signal received"); });

    spin(node);
    shutdown();
    return 0;
}