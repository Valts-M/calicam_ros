#pragma once

#include <memory>
// #include <opencv2/opencv.hpp>

#include "calibration.hpp"

namespace calicam {

class StereoRectifier {
public:
    StereoRectifier(){};
    StereoRectifier(const Calibration& calibration, double fov);

    void undistortRectify(const cv::Mat& in_img_l, const cv::Mat& in_img_r,
                 cv::Mat& out_img_l, cv::Mat& out_img_r) const;

    void lUndistort(const cv::Mat& inImg, cv::Mat& outImg) const;
    void rUndistort(const cv::Mat& inImg, cv::Mat& outImg) const;

    void updateFOV(double fov);

    cv::Mat getP();

private:
    void initRectifyMap();
    Calibration calib;
    void initUndistortRectifyMap(cv::Mat& K, cv::Mat& D, cv::Mat& xi, cv::Mat& R, 
                             cv::Mat& P, cv::Size& size, 
                             cv::Mat& map1, cv::Mat& map2);
    inline double MatRowMul(cv::Mat& m, double x, double y, double z, int r) 
    {
        return m.at<double>(r,0) * x + m.at<double>(r,1) * y + m.at<double>(r,2) * z;
    }

    double fovDeg;
    double fovRad;
    double focal;
    cv::Mat P;

};

} // namespace slambox
