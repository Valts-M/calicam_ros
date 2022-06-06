#include "rectifier_node.hpp"
namespace calicam
{

using namespace std::chrono_literals;
using std::placeholders::_1;

RectifierNode::RectifierNode() : Node("rectifier_node") 
{
    calibFile = declare_parameter("calibration_file", "");
    fov = declare_parameter("fov", 100.0);
    monochrome = declare_parameter("monochrome", false);

    cv::setNumThreads(4);

    if(calibFile.empty())
    {
        RCLCPP_FATAL(get_logger(), "CALIBRATION FILE PATH IS EMPTY");
        exit(-1);
    }

    try
    {
        calib = Calibration(calibFile);
    }
    catch(const std::runtime_error& e)
    {
        RCLCPP_FATAL(get_logger(), e.what());
        exit(-1);
    }

    rectifier = StereoRectifier(calib, fov);

    lRectImgPub = create_publisher<sensor_msgs::msg::Image>("calicam/left/image_rect", 10);
    lImgSub = create_subscription<sensor_msgs::msg::Image>("calicam/left/image_raw", 10, std::bind(&RectifierNode::lImgCallback, this, _1));

    rRectImgPub = create_publisher<sensor_msgs::msg::Image>("calicam/right/image_rect", 10);
    rImgSub = create_subscription<sensor_msgs::msg::Image>("calicam/right/image_raw", 10, std::bind(&RectifierNode::rImgCallback, this, _1));

    RCLCPP_INFO(get_logger(), "Starting rectifier node");
}

  
void RectifierNode::lImgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto img = cv_bridge::toCvCopy(msg);
    if(monochrome)
    {
        cv::cvtColor(img->image, img->image, cv::COLOR_BGR2GRAY);
        img->encoding = "mono8";
    }
    rectifier.lUndistort(img->image, img->image);
    sensor_msgs::msg::Image outMsg;
    img->toImageMsg(outMsg);
    lRectImgPub->publish(outMsg);
}

void RectifierNode::rImgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto img = cv_bridge::toCvCopy(msg);
    if(monochrome)
    {
        cv::cvtColor(img->image, img->image, cv::COLOR_BGR2GRAY);
        img->encoding = "mono8";
    }
    rectifier.rUndistort(img->image, img->image);
    sensor_msgs::msg::Image outMsg;
    img->toImageMsg(outMsg);
    rRectImgPub->publish(outMsg);
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<calicam::RectifierNode>());
  rclcpp::shutdown();
  return 0;
}