//-------------------------------------------------------------------------------------------
/*! \file    template_std.cpp
    \brief   Template of a FingerVision video processing ROS node.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Aug.20, 2018
*/
//-------------------------------------------------------------------------------------------
#include "ay_vision/vision_util.h"
#include "ay_cpp/sys_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace trick;
//-------------------------------------------------------------------------------------------

cv::Mat Capture(cv::VideoCapture &cap, TCameraInfo &info, TCameraRectifier *pcam_rectifier=NULL)
{
  cv::Mat frame;
  while(!cap.read(frame))
  {
    if(CapWaitReopen(info,cap)) continue;
    else  return cv::Mat();
  }
  if(info.CapWidth!=info.Width || info.CapHeight!=info.Height)
    cv::resize(frame,frame,cv::Size(info.Width,info.Height));
  Rotate90N(frame,frame,info.NRotate90);
  if(info.Rectification && pcam_rectifier)
    pcam_rectifier->Rectify(frame, /*border=*/cv::Scalar(0,0,0));
  return frame;
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "template_std");
  ros::NodeHandle node("~");
  std::string cam_config("config/fv_1.yaml");
  std::string vout_base("/tmp/vout-");

  node.param("cam_config",cam_config,cam_config);
  node.param("vout_base",vout_base,vout_base);
  std::cerr<<"cam_config: "<<cam_config<<std::endl;

  std::vector<TCameraInfo> cam_info;
  if(FileExists(cam_config))
    ReadFromYAML(cam_info, cam_config);
  else
  {
    std::cerr<<"Cannot open cam_config: "<<cam_config<<std::endl;
    return -1;
  }

  cv::VideoCapture cap;
  if(!CapOpen(cam_info[0], cap))  return -1;

  // Creating an image rectification object.
  TCameraRectifier cam_rectifier;
  if(cam_info[0].Rectification)
  {
    cv::Size size_in(cam_info[0].Width,cam_info[0].Height), size_out(cam_info[0].Width,cam_info[0].Height);
    cam_rectifier.Setup(cam_info[0].K, cam_info[0].D, cam_info[0].R, size_in, cam_info[0].Alpha, size_out);
  }

  // Video recording tool.
  TEasyVideoOut vout;
  vout.SetfilePrefix("/tmp/cam");

  // [[[Setup publisher, service server, etc.
  // pub= ...
  // Done.]]]

  bool running(true);
  cv::Mat frame;
  for(int f(0);ros::ok();++f)
  {
    if(running)
    {
      frame= Capture(cap, cam_info[0], &cam_rectifier);

      // [[[Process the image:
      // processing frame...
      // Done]]]

      // [[[Publish the result:
      // pub.publish(...);
      // Done]]]

      // Video recording code (necessary even when video is not recorded):
      vout.Step(frame);
      vout.VizRec(frame);  // Draw a recording mark when video is recorded.
      // Displaying image:
      cv::imshow("camera", frame);

    }  // running
    else
    {
      usleep(200*1000);
    }

    // Keyboard event handling:
    char c(cv::waitKey(10));
    if(c=='\x1b'||c=='q') break;
    else if(char(c)=='W')  vout.Switch();  // Start/stop video recording.
    else if(char(c)==' ')
    {
      running=!running;
      std::cerr<<(running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
    }

    ros::spinOnce();
  }

  return 0;
}
//-------------------------------------------------------------------------------------------
