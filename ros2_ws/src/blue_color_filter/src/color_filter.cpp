#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/qos.hpp>

class ColorFilterNode : public rclcpp::Node
{
public:
    ColorFilterNode() : Node("color_filter")
    {
        this->declare_parameter("h_low", 35);
        this->declare_parameter("h_high", 85);
        this->declare_parameter("s_low", 50);
        this->declare_parameter("s_high", 255);
        this->declare_parameter("v_low", 50);
        this->declare_parameter("v_high", 255);

        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/webcam/image_raw", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            std::bind(&ColorFilterNode::image_callback, this, std::placeholders::_1));        

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/filtered/image", 10);
        overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/filtered/overlay", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame, hsv, mask, filtered, black_screen, overlayed;

        // Convert ROS Image to OpenCV format
        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
            return;
        }

        // Convert image to HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Get parameters
        int h_low = this->get_parameter("h_low").as_int();
        int h_high = this->get_parameter("h_high").as_int();
        int s_low = this->get_parameter("s_low").as_int();
        int s_high = this->get_parameter("s_high").as_int();
        int v_low = this->get_parameter("v_low").as_int();
        int v_high = this->get_parameter("v_high").as_int();

        // Create mask for the specified color range
        cv::inRange(hsv, cv::Scalar(h_low, s_low, v_low), cv::Scalar(h_high, s_high, v_high), mask);

        // Extract the color and set to red (0,0,255)
        filtered = cv::Mat::zeros(frame.size(), frame.type());
        filtered.setTo(cv::Scalar(0, 0, 255), mask); // Change detected color to red

        // Create black screen for filtered image
        black_screen = cv::Mat::zeros(frame.size(), frame.type());
        filtered.copyTo(black_screen, mask);

        // Overlay the filtered color onto the original image
        overlayed = frame.clone();
        cv::addWeighted(filtered, 0.5, overlayed, 1.0, 0, overlayed); // Blend filtered image onto original

        // Convert to ROS messages and publish
        auto filtered_msg = cv_bridge::CvImage(msg->header, "bgr8", black_screen).toImageMsg();
        auto overlayed_msg = cv_bridge::CvImage(msg->header, "bgr8", overlayed).toImageMsg();
        
        cv::imshow("Filtered Image", black_screen);
        cv::imshow("Overlayed Image", overlayed);

        image_pub_->publish(*filtered_msg);
        overlay_pub_->publish(*overlayed_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorFilterNode>());
    rclcpp::shutdown();
    return 0;
}
