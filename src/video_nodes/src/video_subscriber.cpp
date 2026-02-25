#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace video_sub_ns
{
// read ros names
using img_msg = sensor_msgs::msg::Image;
using img_sub = rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr;
using img_ptr = img_msg::SharedPtr;

// adding ros names
using cv_bridge::CvImagePtr;
using cv_bridge::Exception;
using cv_bridge::toCvCopy;
using rclcpp::init;
using rclcpp::Node;
using rclcpp::SensorDataQoS;
using rclcpp::shutdown;
using rclcpp::spin;

// adding opencv names
using cv::destroyAllWindows;
using cv::imshow;
using cv::Mat;
using cv::waitKey;

// adding std name
using std::make_shared;
using std::string;
using std::chrono::milliseconds;
using namespace std::placeholders;

}  // namespace video_sub_ns
using namespace video_sub_ns;
class VideoSubscriber : public Node
{
   public:
    VideoSubscriber() : Node("video_subscriber")
    {
        // Create subscription for raw images
        sub_ = this->create_subscription<img_msg>("camera/image_raw",
                                                  SensorDataQoS(),  // Best effort, low latency
                                                  bind(&VideoSubscriber::image_callback, this, _1));

        // Create display window
        namedWindow("Camera Feed", cv::WINDOW_NORMAL);

        RCLCPP_INFO(this->get_logger(), "Video subscriber started - press 'q' to exit");
    }

    ~VideoSubscriber()
    {
        destroyAllWindows();
    }

   private:
    img_sub sub_;

    void image_callback(const img_ptr msg)
    {
        try
        {
            // Convert ROS image to OpenCV
            CvImagePtr cv_ptr = toCvCopy(msg, "bgr8");

            // Display the frame immediately
            imshow("Camera Feed", cv_ptr->image);

            // Check for exit key
            int key = waitKey(1);
            if (key == 'q' || key == 'Q')
            {
                RCLCPP_INFO(this->get_logger(), "Shutting down by user request");
                shutdown();
            }
        }
        catch (const Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    init(argc, argv);
    auto node = make_shared<VideoSubscriber>();
    spin(node);
    shutdown();
    return 0;
}