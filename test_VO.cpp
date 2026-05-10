#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
#include "test_utils.h"

// note frame->T_f_w = pose as Sophus::SE3
// note frame->fts_ = gives feature list
// note DepthFilter() to access depth filter

namespace svo {

class CameraVO
{
    vk::AbstractCamera* cam_;
    svo::FrameHandlerMono* vo_;


public:
    CameraVO();
    void runVO();
};

CameraVO::CameraVO()
{
    cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
    vo_ = new svo::FrameHandlerMono(cam_);
    vo_->start();
}

CameraVO::~CameraVO()
{
  delete vo_;
  delete cam_;
}

CameraVO::runVO(cv::Mat &img)
{
    vo_->addImage(img);
}



}

int main(int argc, char** argv)
{

}