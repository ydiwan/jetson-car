#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_base.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <utility>

#include "../include/lane_detection/lane_detector.h"
/// custom namespace for readability
/// /////////////////////////////////////////////////////////
namespace lane_detection_node_ns {
// rename types for readability
// rename image
using img_msg = sensor_msgs::msg::Image;
using img_ptr = img_msg::SharedPtr;
using img_pub = rclcpp::Publisher<img_msg>::SharedPtr;
using img_sub = rclcpp::Subscription<img_msg>::SharedPtr;

// rename int
using int_msg = std_msgs::msg::Int32;
using int_pub = rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr;

// rename double
using double_msg = std_msgs::msg::Float64;
using double_pub = rclcpp::Publisher<double_msg>::SharedPtr;
// end of renaming

// adding ros2
using cv_bridge::CvImage;
using rclcpp::init;
using rclcpp::SensorDataQoS;
using rclcpp::shutdown;
using rclcpp::spin;
using Header = std_msgs::msg::Header_<std::allocator<void>>;
using timer_ptr = rclcpp::TimerBase::SharedPtr;

// opencv
using cv::Mat;
using cv::Point;
using cv::Point2f;

// adding std
using std::bind;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using namespace std::placeholders;
using std::make_shared;
using std::chrono::milliseconds;

// adding lane detector namespace
using ld_ns::Lane_detector;
};  // namespace lane_detection_node_ns
using namespace lane_detection_node_ns;

/// Class Node Declaration
/// /////////////////////////////////////////////////////////
class lane_detection_node : public rclcpp::Node {
   public:
    lane_detection_node() : Node("lane_detection_node") {
        // contruct lane_detector class and pass this node as a arg
        ld_ = std::make_unique<Lane_detector>(this);

        // Declare parameters for the camera
        this->declare_parameter("camera_index", 0);  // index of the camera
        this->declare_parameter("width", 640);       // width of the cam feed
        this->declare_parameter("height", 480);      // height of the cam feed
        this->declare_parameter("fps", 60);          // fps of the cam feed

        // Get parameters for camera
        int cam_index = this->get_parameter("camera_index").as_int();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();

        // Create the working pipeline
        string pipeline = create_pipeline(cam_index, width, height, fps);

        cap_ = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera with pipeline: %s",
                         pipeline.c_str());
            return;
        }

        // setting up publishers
        raw_video_pub_ = this->create_publisher<img_msg>("camera/raw_video", 1);
        result_img_pub_ =
            this->create_publisher<img_msg>("camera/lane_detection_result", 1);
        bev_img_pub_ = this->create_publisher<img_msg>("camera/bev", 1);
        mask_img_pub_ = this->create_publisher<img_msg>("camera/bin_mask", 1);
        filter_img_pub_ = this->create_publisher<img_msg>("camera/filter_bin_mask", 1);
        delta_pub_ = this->create_publisher<int_msg>("lane_detect/delta", 1);
        position_confidence_pub_ =
            this->create_publisher<double_msg>("lane_detect/position_confidence", 1);
        symmetry_confidence_pub_ =
            this->create_publisher<double_msg>("lane_detect/symmetry_confidence", 1);

        RCLCPP_INFO(this->get_logger(), "lane_detection_node has started");

        timer_ = this->create_wall_timer(
            milliseconds(int(1 / fps)), bind(&lane_detection_node::process_frame, this));
    }

   private:
    // publisher objects
    img_pub raw_video_pub_;
    img_pub mask_img_pub_;
    img_pub filter_img_pub_;
    img_pub result_img_pub_;
    img_pub bev_img_pub_;
    int_pub delta_pub_;
    double_pub position_confidence_pub_;
    double_pub symmetry_confidence_pub_;

    // lane detector object
    std::unique_ptr<Lane_detector> ld_;

    // videoCapture object
    cv::VideoCapture cap_;

    // callback timer
    timer_ptr timer_;

    void pub_img(img_pub pub, Mat img, string encoding) {
        img_ptr temp_ptr = CvImage(Header(), encoding, img).toImageMsg();
        temp_ptr->header.stamp = this->get_clock()->now();
        temp_ptr->header.frame_id = "video_feed";

        // publish  result
        pub->publish(*temp_ptr);
    }

    void process_frame() {
        int_msg delta;
        double_msg position_confidence;
        double_msg symmetry_confidence;
        Mat frame;

        // get frame and process it
        cap_ >> frame;

        if (!frame.empty()) {
            if (frame.channels() == 4) {
                // need to convert the BGRx to BGR
                cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
            }
            auto results = ld_->step(frame);

            delta.data = results.delta;
            RCLCPP_DEBUG(this->get_logger(), "delta: %d", delta.data);

            // get confidence values
            position_confidence.data = results.position_conf;
            symmetry_confidence.data = results.symmetry_conf;

            // pub servo value and confidence values
            delta_pub_->publish(delta);
            position_confidence_pub_->publish(position_confidence);
            symmetry_confidence_pub_->publish(symmetry_confidence);

            // pub images for lane detection algorithm
            RCLCPP_DEBUG(this->get_logger(), "pub images");
            pub_img(raw_video_pub_, frame, "bgr8");
            pub_img(result_img_pub_, results.ld_img, "bgr8");
            pub_img(bev_img_pub_, results.bev, "bgr8");
            pub_img(mask_img_pub_, results.bin_mask, "mono8");
            pub_img(filter_img_pub_, results.filter_mask, "mono8");
        }
    }

    string create_pipeline(int cam_index, int width, int height, int fps) {
        std::ostringstream pipeline;

        pipeline << "v4l2src device=/dev/video" << cam_index
                 << " ! video/x-raw,width=(int)" << width << ",height=(int)"
                 << height << ",framerate=(fraction)" << fps << "/1 "
                 << "! nvvidconv"
                 << "! video/x-raw,format=BGRx "  // Specify output format
                 << "! videoconvert "             // Ensure proper conversion
                 << "! video/x-raw,format=BGR "   // Convert to BGR for OpenCV
                 << "! appsink sync=false";

        RCLCPP_INFO(this->get_logger(), "Pipeline: %s", pipeline.str().c_str());
        return pipeline.str();
    }
};

/// Main ////////////////////////////////////////////////////
int main(int argc, char **argv) {
    init(argc, argv);
    auto node = make_shared<lane_detection_node>();
    spin(node);
    shutdown();
    return 0;
}