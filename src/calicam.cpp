#include "calicam.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace calicam
{

using namespace std::chrono_literals;

CaliCam::CaliCam() : Node("calicam") 
{
    calibFile = declare_parameter("calibration_file", "");
    fov = declare_parameter("fov", 100.0);
    fps = declare_parameter("fps", 30.0);
    undirstortRectify = declare_parameter("undistort_rectify", false);
    cameraIndex = declare_parameter("camera_index", 0);
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

    lInfo = generateCameraInfo(true);
    if(calib.stereo)
        rInfo = generateCameraInfo(false);

    lCvBridge.encoding = monochrome ? sensor_msgs::image_encodings::TYPE_8UC1 : sensor_msgs::image_encodings::TYPE_8UC3;
    rCvBridge.encoding = monochrome ? sensor_msgs::image_encodings::TYPE_8UC1 : sensor_msgs::image_encodings::TYPE_8UC3;

    lCvBridge.header.frame_id = "calicam_left";
    rCvBridge.header.frame_id = "calicam_right";

    // publish static transform between sensor frames
    tf_publisher = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    geometry_msgs::msg::TransformStamped leftTf;
    leftTf.header.frame_id = "calicam";
    leftTf.child_frame_id = lCvBridge.header.frame_id;
    leftTf.header.stamp = now();
    leftTf.transform.rotation.x = 0.5;
    leftTf.transform.rotation.y = -0.5;
    leftTf.transform.rotation.z = 0.5;
    leftTf.transform.rotation.w = -0.5;
    tf_publisher->sendTransform(leftTf);

    if(calib.stereo)
    {
        geometry_msgs::msg::TransformStamped rightTf;
        rightTf.header.frame_id = "calicam";
        rightTf.child_frame_id = rCvBridge.header.frame_id;
        rightTf.header.stamp = leftTf.header.stamp;
        rightTf.transform.rotation.x = 0.5;
        rightTf.transform.rotation.y = -0.5;
        rightTf.transform.rotation.z = 0.5;
        rightTf.transform.rotation.w = -0.5;
        // translation seems to be from the right camera to the left so we flip the values
        // also need to change the axis to ros coordinate system
        rightTf.transform.translation.x = -calib.Translation.at<double>(2, 0);
        rightTf.transform.translation.y = calib.Translation.at<double>(0, 0);
        rightTf.transform.translation.z = calib.Translation.at<double>(1, 0);
        tf_publisher->sendTransform(rightTf);
    }

    lImgPub = create_publisher<sensor_msgs::msg::Image>("calicam/left/image_raw", 10);
    lRectImgPub = create_publisher<sensor_msgs::msg::Image>("calicam/left/image_rect", 10);
    lInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>("calicam/left/camera_info", 10);

    rImgPub = create_publisher<sensor_msgs::msg::Image>("calicam/right/raw_img", 10);
    rRectImgPub = create_publisher<sensor_msgs::msg::Image>("calicam/right/rect_img", 10);
    rInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>("calicam/right/camera_info", 10);

    vCapture.open(cameraIndex);
    vCapture.set(cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH,  calib.capSize.width);
    vCapture.set(cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT, calib.capSize.height);
    vCapture.set(cv::VideoCaptureProperties::CAP_PROP_FPS, fps);

    timer = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / fps)), std::bind(&CaliCam::updateHandler, this));
    RCLCPP_INFO(get_logger(), "Starting calicam");
}

sensor_msgs::msg::CameraInfo CaliCam::generateCameraInfo(const bool leftCamera)
{
    sensor_msgs::msg::CameraInfo info;
    info.height = calib.imgSize.height;
    info.width = calib.imgSize.width;
    info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

    cv::Size size;

    const cv::Mat K = leftCamera ? calib.Kl : calib.Kr;
    size = K.size();
    for(int i = 0; i < size.height; i++)
    {
        for(int j = 0; j < size.width; j++)
        {
            info.k[i * 3 + j] = K.at<double>(i, j);
        }
    }

    const cv::Mat R = leftCamera ? calib.Rl : calib.Rr;
    size = R.size();
    for(int i = 0; i < size.height; i++)
    {
        for(int j = 0; j < size.width; j++)
        {
            info.r[i * 3 + j] = R.at<double>(i, j);
        }
    }

    const cv::Mat D = leftCamera ? calib.Dl : calib.Dr;
    size = D.size();
    for(int i = 0; i < size.height; i++)
    {
        for(int j = 0; j < size.width; j++)
        {
            info.d.push_back(D.at<double>(i, j));
        }
    }

    cv::Mat P = rectifier.getP();
    size = P.size();
    for(int i = 0; i < size.height; i++)
    {
        for(int j = 0; j < size.width + 1; j++)
        {
            if(j == size.width)
            {
                info.p[i * 3 + j] = 0;
                continue;
            }
            info.p[i * 3 + j] = P.at<double>(i, j);
        }
    }
    if(!leftCamera)
    {
        // Tx = -fx * camera_baseline (the baselink in the calibration is already negative)
        info.p[3] = info.p[0] * calib.Translation.at<double>(0, 0);
    }
    return info;
}

  
void CaliCam::resetCamera()
{
    vCapture.release();
    vCapture.open(cameraIndex);
    vCapture.set(cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH,  calib.capSize.width);
    vCapture.set(cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT, calib.capSize.height);
    vCapture.set(cv::VideoCaptureProperties::CAP_PROP_FPS, fps);
}

void CaliCam::updateHandler()
{
    if(!vCapture.isOpened())
    {
        RCLCPP_ERROR(get_logger(), "CAN'T CONNECTO TO CAMERA %i", cameraIndex);
        resetCamera();
        return;
    }

    vCapture >> latestFrame;
    lCvBridge.header.stamp = now();
    rCvBridge.header.stamp = lCvBridge.header.stamp;

    if(latestFrame.total() == 0)
    {
        RCLCPP_WARN(get_logger(), "RECEIVED EMPTY IMAGE");
        resetCamera();
        return;
    }

    if(monochrome) cv::cvtColor(latestFrame, latestFrame, cv::COLOR_BGR2GRAY);

    if(calib.stereo)
    {
        latestFrame(cv::Rect(0, 0, calib.imgSize.width, calib.imgSize.height)).copyTo(lCvBridge.image);
        latestFrame(cv::Rect(calib.imgSize.width, 0, calib.imgSize.width, calib.imgSize.height)).copyTo(rCvBridge.image);

        lCvBridge.toImageMsg(lMsg);
        rCvBridge.toImageMsg(rMsg);

        lInfo.header = lMsg.header;
        rInfo.header = rMsg.header;

        lInfoPub->publish(lInfo);
        rInfoPub->publish(rInfo);

        lImgPub->publish(lMsg);
        rImgPub->publish(rMsg);

        if(!undirstortRectify) return;

        rectifier.undistortRectify(lCvBridge.image, rCvBridge.image, lCvBridge.image, rCvBridge.image);

        lCvBridge.toImageMsg(lMsg);
        rCvBridge.toImageMsg(rMsg);

        lRectImgPub->publish(lMsg);
        rRectImgPub->publish(rMsg);
    }
    else
    {
        lCvBridge.image = latestFrame;
        lCvBridge.toImageMsg(lMsg);

        lInfo.header = lMsg.header;
        lInfoPub->publish(lInfo);
        
        lImgPub->publish(lMsg);

        if(!undirstortRectify) return;

        rectifier.lUndistort(lCvBridge.image, lCvBridge.image);
        lCvBridge.toImageMsg(lMsg);
        lRectImgPub->publish(lMsg);
    }

}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<calicam::CaliCam>());
  rclcpp::shutdown();
  return 0;
}