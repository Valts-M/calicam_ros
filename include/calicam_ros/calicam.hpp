#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>

#include "calibration.hpp"
#include "stereo_rectifier.hpp"

namespace calicam
{
class CaliCam : public rclcpp::Node
{
public:
    CaliCam();

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lImgPub{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lRectImgPub{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr lInfoPub{nullptr};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rImgPub{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rRectImgPub{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rInfoPub{nullptr};

    rclcpp::TimerBase::SharedPtr timer;

    void resetCamera();
    void updateHandler();

    sensor_msgs::msg::CameraInfo generateCameraInfo(const bool leftCamera);

    Calibration calib;
    std::string calibFile;
    double fov;
    double fps;
    int cameraIndex;
    bool undirstortRectify;
    bool monochrome;

    cv::VideoCapture vCapture;
    cv::Mat latestFrame, lImg, rImg;
    StereoRectifier rectifier;

    cv_bridge::CvImage lCvBridge, rCvBridge;
    sensor_msgs::msg::Image lMsg, rMsg;
    sensor_msgs::msg::CameraInfo lInfo, rInfo;
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher;
};
}