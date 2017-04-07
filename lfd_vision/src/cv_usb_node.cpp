//-------------------------------------------------------------------------------------------
/*! \file    cv_usb_node.cpp
    \brief   USB camera node for multiple cameras with OpenCV.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.08, 2016
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/vision_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cstdio>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
//-------------------------------------------------------------------------------------------
namespace trick
{
}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

bool OpenCameras(std::vector<TCameraInfo> &cam_info, std::vector<cv::VideoCapture> &cap)
{
  for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
  {
    if(!CapOpen(cam_info[i_cam], cap[i_cam]))  return -1;
    cv::namedWindow(cam_info[i_cam].Name,1);
  }
  std::cerr<<"Opened camera(s)"<<std::endl;
  return true;
}

bool CaptureTest(std::vector<cv::VideoCapture> &cap)
{
  std::vector<cv::Mat> frame(cap.size());
  std::cerr<<"Capture test..."<<std::endl;
  for(int i_cam(0), i_cam_end(cap.size()); i_cam<i_cam_end; ++i_cam)
  {
    // cap[i_cam] >> frame[i_cam];
    bool res= cap[i_cam].read(frame[i_cam]);
    if(!res)
    {
      std::cerr<<" Failed."<<std::endl;
      return false;
    }
  }
  std::cerr<<" OK."<<std::endl;
  return true;
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "cv_usb_node");
  ros::NodeHandle node("~");
  std::string config_file("config/usb_cams4g.yaml");

  if(argc>1)  config_file= argv[1];

  std::vector<TCameraInfo> cam_info;
  ReadFromYAML(cam_info, config_file);

  std::vector<cv::VideoCapture> cap(cam_info.size());
  bool cap_failure(true);
  for(int i(0);i<5;++i)
  {
    if(!OpenCameras(cam_info, cap))  return -1;
    if(CaptureTest(cap))  {cap_failure=false; break;}
    for(int i_cam(0), i_cam_end(cap.size()); i_cam<i_cam_end; ++i_cam)
      cap[i_cam].release();
    cap.clear();
    cap.resize(cam_info.size());
    usleep(200*1000);
  }
  if(cap_failure)  return -1;

  std::vector<TEasyVideoOut> video_out;
  video_out.resize(cam_info.size());
  for(int j(0),j_end(video_out.size());j<j_end;++j)
    video_out[j].SetfilePrefix("/tmp/vout");

  image_transport::ImageTransport imgtr(node);
  std::vector<image_transport::Publisher> pub;
  typedef boost::shared_ptr<camera_info_manager::CameraInfoManager> CamInfoMngrPtr;
  std::vector<CamInfoMngrPtr> info_manager;
  for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
  {
    pub.push_back(imgtr.advertise(cam_info[i_cam].Name+"/image_raw", 1));
    info_manager.push_back(CamInfoMngrPtr(new camera_info_manager::CameraInfoManager(ros::NodeHandle("~/"+cam_info[i_cam].Name), cam_info[i_cam].Name, /*camera_info_url=*/"")));
  }

  std::vector<cv::Mat> frame(cam_info.size());
  int show_fps(0);
  for(;ros::ok();)
  {
    /*FIXME: Capture should be a thread per camera to handle bigger FPS (e.g. 60).
    FIXME: Display FPS should be smaller e.g. 30 (use ROS rate adjuster). */

    // Capture from cameras:
    bool failure(false);
    for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
    {
      // cap[i_cam] >> frame[i_cam];
      bool res= cap[i_cam].read(frame[i_cam]);
      if(!res)  {failure=true; break;}
    }
    if(failure)  {usleep(200*1000); continue;}

    // Image processing:
    for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
    {
      if(cam_info[i_cam].CapWidth!=cam_info[i_cam].Width || cam_info[i_cam].CapHeight!=cam_info[i_cam].Height)
        cv::resize(frame[i_cam],frame[i_cam],cv::Size(cam_info[i_cam].Width,cam_info[i_cam].Height));
      Rotate90N(frame[i_cam],frame[i_cam],cam_info[i_cam].NRotate90);
    }

    // Publish images:
    for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
      pub[i_cam].publish( cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame[i_cam]).toImageMsg() );

    // Video output & Display images:
    for(int i_cam(0), i_cam_end(cam_info.size()); i_cam<i_cam_end; ++i_cam)
    {
      video_out[i_cam].Step(frame[i_cam]);
      video_out[i_cam].VizRec(frame[i_cam]);
      cv::imshow(cam_info[i_cam].Name, frame[i_cam]);
    }

    if(show_fps==0)
    {
      std::cerr<<"FPS: "<<video_out[0].FPS()<<std::endl;
      show_fps=video_out[0].FPS()*4;
    }
    --show_fps;

    char c(cv::waitKey(1));
    if(c=='\x1b'||c=='q') break;
    else if(c=='W')
    {
      for(int j(0),j_end(video_out.size());j<j_end;++j)
        video_out[j].Switch();
    }
    // usleep(10000);
  }
  return 0;
}
//-------------------------------------------------------------------------------------------
