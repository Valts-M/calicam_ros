#include "stereo_rectifier.hpp"

#include <opencv2/imgproc.hpp>

namespace calicam {

StereoRectifier::StereoRectifier(const Calibration& calibration, const double fov) : calib(calibration)
{
  updateFOV(fov);
}

void StereoRectifier::updateFOV(const double fov)
{
  fovDeg = fov;
  fovRad = fovDeg * CV_PI / 180.;
  focal = calib.imgSize.height / 2. / tan(fovRad / 2.);
  P = (cv::Mat_<double>(3, 3) << 
          focal, 0., calib.imgSize.width  / 2.,
          0., focal, calib.imgSize.height / 2.,
          0., 0., 1.);
  initRectifyMap();
}

cv::Mat StereoRectifier::getP()
{
  return P;
}

void StereoRectifier::undistortRectify(const cv::Mat& in_img_l, const cv::Mat& in_img_r,
    cv::Mat& out_img_l, cv::Mat& out_img_r) const 
{
  cv::remap(in_img_l, out_img_l, calib.smap[0][0], calib.smap[0][1], cv::InterpolationFlags::INTER_LINEAR);
  cv::remap(in_img_r, out_img_r, calib.smap[1][0], calib.smap[1][1], cv::InterpolationFlags::INTER_LINEAR);
}

void StereoRectifier::undistort(const cv::Mat& inImg, cv::Mat& outImg) const
{
  cv::remap(inImg, outImg, calib.smap[0][0], calib.smap[0][1], cv::InterpolationFlags::INTER_LINEAR);
}

void StereoRectifier::initUndistortRectifyMap(cv::Mat& K, cv::Mat& D, cv::Mat& xi, cv::Mat& R, 
                             cv::Mat& P, cv::Size& size, 
                             cv::Mat& map1, cv::Mat& map2) {
  map1 = cv::Mat(size, CV_32F);
  map2 = cv::Mat(size, CV_32F);

  const double fx = K.at<double>(0,0);
  const double fy = K.at<double>(1,1);
  const double cx = K.at<double>(0,2);
  const double cy = K.at<double>(1,2);
  const double s  = K.at<double>(0,1);

  const double xid = xi.at<double>(0,0);

  const double k1 = D.at<double>(0,0);
  const double k2 = D.at<double>(0,1);
  const double p1 = D.at<double>(0,2);
  const double p2 = D.at<double>(0,3);

  cv::Mat KRi = (P * R).inv();

  for (int r = 0; r < size.height; ++r) {
    for (int c = 0; c < size.width; ++c) {
      const double xc = MatRowMul(KRi, c, r, 1., 0);
      const double yc = MatRowMul(KRi, c, r, 1., 1);
      const double zc = MatRowMul(KRi, c, r, 1., 2);    

      const double rr = sqrt(xc * xc + yc * yc + zc * zc);
      const double xs = xc / rr;
      const double ys = yc / rr;
      const double zs = zc / rr;

      const double xu = xs / (zs + xid);
      const double yu = ys / (zs + xid);

      const double r2 = xu * xu + yu * yu;
      const double r4 = r2 * r2;
      const double xd = (1+k1*r2+k2*r4)*xu + 2*p1*xu*yu + p2*(r2+2*xu*xu);
      const double yd = (1+k1*r2+k2*r4)*yu + 2*p2*xu*yu + p1*(r2+2*yu*yu);
 
      const double u = fx * xd + s * yd + cx;
      const double v = fy * yd + cy;

      map1.at<float>(r,c) = (float) u;
      map2.at<float>(r,c) = (float) v;
    }
  }
}

void StereoRectifier::initRectifyMap() 
{

  initUndistortRectifyMap(calib.Kl, calib.Dl, calib.xil, calib.Rl, P, 
                          calib.imgSize, calib.smap[0][0], calib.smap[0][1]);

  if (calib.stereo) 
  {
    initUndistortRectifyMap(calib.Kr, calib.Dr, calib.xir, calib.Rr, P, 
                            calib.imgSize, calib.smap[1][0], calib.smap[1][1]);
  }
}


} // namespace openvslam
