#pragma once
#include <opencv2/opencv.hpp>
#include <exception>

namespace calicam
{
struct Calibration
{
    cv::Mat Translation, Kl, Kr, Dl, Dr, xil, xir, Rl, Rr, smap[2][2];
    cv::Size capSize, imgSize;
    cv::Size defaultImgSize{1280, 960};
    bool stereo;

    Calibration(){};

    Calibration(const std::string& filename) 
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) 
        {
            throw std::runtime_error("Couldn't open file: " + filename);
        }

        std::string camModel;
        fs["cam_model"] >> camModel;
        stereo = camModel == "stereo";

        fs["cap_size" ] >> capSize;
        fs["Kl"       ] >> Kl;
        fs["Dl"       ] >> Dl;
        fs["xil"      ] >> xil;
        Rl = cv::Mat::eye(3, 3, CV_64F);
        imgSize = capSize;

        if (stereo) {
            fs["Rl"       ] >> Rl;
            fs["Kr"       ] >> Kr;
            fs["Dr"       ] >> Dr;
            fs["xir"      ] >> xir;
            fs["Rr"       ] >> Rr;
            fs["T"        ] >> Translation;
            imgSize.width = imgSize.width / 2.;
        }

        if(imgSize != defaultImgSize)
        {
            const double scaleHeight = static_cast<double>(imgSize.height) / static_cast<double>(defaultImgSize.height);
            const double scaleWidth = static_cast<double>(imgSize.width) / static_cast<double>(defaultImgSize.width);
            Kl.row(0) = Kl.row(0) * scaleWidth;
            Kl.row(1) = Kl.row(1) * scaleHeight;
            if(stereo)
            {
                Kr.row(0) = Kr.row(0) * scaleWidth;
                Kr.row(1) = Kr.row(1) * scaleHeight;
            }
        }

        fs.release();
    }
};
}