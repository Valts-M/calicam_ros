#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

#include "calibration.hpp"
#include "stereo_rectifier.hpp"

namespace calicam
{
class RectifierNode : public rclcpp::Node
{
public:
    RectifierNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr lImgSub{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lRectImgPub{nullptr};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rImgSub{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rRectImgPub{nullptr};

    void lImgCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void rImgCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    Calibration calib;
    std::string calibFile;
    double fov;
    bool monochrome;

    StereoRectifier rectifier;

};
}