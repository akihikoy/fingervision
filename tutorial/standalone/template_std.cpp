//-------------------------------------------------------------------------------------------
/*! \file    template_std.cpp
    \brief   Template of a standalone FingerVision video processing program.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Aug.20, 2018

Compile:
  g++ -g -Wall -O2 -o template_std.out template_std.cpp -I../../3rdparty -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_highgui

Run:
  $ ./template_std.out [CAMERA_NUMBER [WIDTH HEIGHT]]
    CAMERA_NUMBER: Camera device number (e.g. 1).
  $ ./template_std.out CONFIG_FILE
    CONFIG_FILE: Configuration file in YAML (e.g. fv_1.yaml).
    The configuration file may include camera configuration (CameraInfo).
Usage:
  Press 'q' or Esc: Exit the program.
  Press 'W' (shift+'w'): Start/stop video recording.
*/
//-------------------------------------------------------------------------------------------
#include "ay_vision/vision_util.h"
#include "ay_cpp/sys_util.h"
//-------------------------------------------------------------------------------------------
#include "ay_vision/vision_util.cpp"
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
  std::string cam("0");
  if(argc>1)  cam= argv[1];

  std::vector<TCameraInfo> cam_info;
  // If cam exists as a file, it is considered as a configuration YAML file.
  if(FileExists(cam))
  {
    ReadFromYAML(cam_info, cam);
  }
  else
  {
    cam_info.push_back(TCameraInfo());
    cam_info[0].DevID= cam;
    cam_info[0].Width= ((argc>2)?atoi(argv[2]):0);
    cam_info[0].Height= ((argc>3)?atoi(argv[3]):0);
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

  std::string win("camera");
  cv::namedWindow(win,1);

  // Video recording tool.
  TEasyVideoOut vout;
  vout.SetfilePrefix("/tmp/cam");

  cv::Mat frame;
  for(;;)
  {
    frame= Capture(cap, cam_info[0], &cam_rectifier);

    // [[[Process the image:
    // processing frame...
    // Done]]]

    // Video recording code (necessary even when video is not recorded):
    vout.Step(frame);
    vout.VizRec(frame);  // Draw a recording mark when video is recorded.
    // Displaying image:
    cv::imshow("camera", frame);

    // Keyboard event handling:
    char c(cv::waitKey(10));
    if(c=='\x1b'||c=='q') break;
    else if(char(c)=='W')  vout.Switch();  // Start/stop video recording.
  }

  return 0;
}
//-------------------------------------------------------------------------------------------
