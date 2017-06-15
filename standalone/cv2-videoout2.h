//-------------------------------------------------------------------------------------------
/*! \file    cv2-videoout2.h
    \brief   Easy video output tool.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.06, 2016
*/
//-------------------------------------------------------------------------------------------
#ifndef cv2_videoout2_h
#define cv2_videoout2_h
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <cstdio>
#include <sys/time.h>  // gettimeofday
#include <fstream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

inline double GetCurrentTime()
{
  struct timeval time;
  gettimeofday(&time, NULL);
  return static_cast<double>(time.tv_sec) + static_cast<double>(time.tv_usec)*1.0e-6;
}
inline bool FileExists(const std::string &filename)
{
  bool res(false);
  std::ifstream ifs (filename.c_str());
  res = ifs.is_open();
  ifs.close();
  return res;
}
//-------------------------------------------------------------------------------------------

bool OpenVideoOut(cv::VideoWriter &vout, const char *file_name, int fps, const cv::Size &size)
{
  // int codec= CV_FOURCC('P','I','M','1');  // mpeg1video
  // int codec= CV_FOURCC('X','2','6','4');  // x264?
  int codec= CV_FOURCC('m','p','4','v');  // mpeg4 (Simple Profile)
  vout.open(file_name, codec, fps, size, true);

  if (!vout.isOpened())
  {
    std::cout<<"###Failed to open the output video: "<<file_name<<std::endl;
    return false;
  }
  std::cerr<<"###Opened video output: "<<file_name<<std::endl;
  return true;
}
//-------------------------------------------------------------------------------------------


/* Easy video output tool that automatically measures FPS.
    Filename is automatically decided with a sequential index.

  Example:
    TEasyVideoOut vout;
    vout.SetfilePrefix("/tmp/vout");
    while(...)
    {
      ...
      vout.Step(frame);
      vout.VizRec(frame);
      // display frame here
      if(key=="W")  vout.Switch();
    }
*/
class TEasyVideoOut
{
public:
  TEasyVideoOut(const double init_fps=10.0);

  // Automatically switch Rec/Stop.
  void Switch()
    {
      if(!writer_.isOpened())  Rec();
      else  Stop();
    }

  // Start recording.
  void Rec();

  // Stop recording.
  void Stop();

  // Writing frame(during recording)/updating FPS,image size.
  void Step(const cv::Mat &frame);

  /* Visualize a recording mark (red circle), effective only during recording.
      pos: position (0: left top (default), 1: left bottom, 2: right bottom, 3: right top)
  */
  void VizRec(cv::Mat &frame, int pos=0, int rad=5, int margin=3) const;

  bool IsRecording() const {return writer_.isOpened();}
  const double& FPS() const {return fps_;}

  void SetfilePrefix(const std::string &v)  {file_prefix_= v;}
  void SetfileSuffix(const std::string &v)  {file_suffix_= v;}

private:
  std::string file_prefix_, file_suffix_;
  cv::VideoWriter writer_;
  cv::Size img_size_;
  double fps_;
  double time_prev_, fps_alpha_;
};
//-------------------------------------------------------------------------------------------

TEasyVideoOut::TEasyVideoOut(const double init_fps)
  :
    file_prefix_ ("/tmp/video"),
    file_suffix_ (".avi"),
    img_size_ (0,0),
    fps_ (init_fps),
    time_prev_ (-1.0),
    fps_alpha_ (0.05)
{
}
//-------------------------------------------------------------------------------------------

// Start recording.
void TEasyVideoOut::Rec()
{
  if(!writer_.isOpened())
  {
    int i(0);
    std::string file_name;
    do
    {
      std::stringstream ss;
      ss<<file_prefix_<<std::setfill('0')<<std::setw(4)<<i<<file_suffix_;
      file_name= ss.str();
      ++i;
    } while(FileExists(file_name));
    OpenVideoOut(writer_, file_name.c_str(), fps_, img_size_);
  }
}
//-------------------------------------------------------------------------------------------

// Stop recording.
void TEasyVideoOut::Stop()
{
  if(writer_.isOpened())
  {
    writer_.release();
    std::cerr<<"###Finished: video output"<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

// Writing frame(during recording)/updating FPS,image size
void TEasyVideoOut::Step(const cv::Mat &frame)
{
  img_size_= cv::Size(frame.cols, frame.rows);

  // update fps
  if(time_prev_<0.0)
  {
    time_prev_= GetCurrentTime();
  }
  else
  {
    double new_fps= 1.0/(GetCurrentTime()-time_prev_);
    if(new_fps>fps_/20.0 && new_fps<fps_*20.0)  // Removing outliers (e.g. pause/resume)
      fps_= fps_alpha_*new_fps + (1.0-fps_alpha_)*fps_;
    time_prev_= GetCurrentTime();
  }

  if(writer_.isOpened())
  {
    if(frame.depth()==CV_8U && frame.channels()==3)
      writer_<<frame;
    else
    {
      // If frame is [0...1] float type matrix:
      cv::Mat frame2,frame3;
      if(frame.depth()!=CV_8U)
        cv::Mat(frame*255.0).convertTo(frame2, CV_8UC(frame.channels()));
      else
        frame2= frame;
      if(frame2.channels()!=3)
      {
        cv::Mat in[]= {frame2, frame2, frame2};
        cv::merge(in, 3, frame3);
      }
      else
        frame3= frame2;
      writer_<<frame3;
    }
  }
}
//-------------------------------------------------------------------------------------------

/* Visualize a recording mark (red circle), effective only during recording.
    pos: position (0: left top (default), 1: left bottom, 2: right bottom, 3: right top)
*/
void TEasyVideoOut::VizRec(cv::Mat &frame, int pos, int rad, int margin) const
{
  if(writer_.isOpened())
  {
    cv::Point2d pt((rad+margin), (rad+margin));
    switch(pos)
    {
    case 0:  pt= cv::Point2d((rad+margin),(rad+margin)); break;
    case 1:  pt= cv::Point2d((rad+margin),frame.rows-(rad+margin)); break;
    case 2:  pt= cv::Point2d(frame.cols-(rad+margin),frame.rows-(rad+margin)); break;
    case 3:  pt= cv::Point2d(frame.cols-(rad+margin),(rad+margin)); break;
    }
    cv::circle(frame, pt, rad, cv::Scalar(0,0,255), -1);
  }
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // cv2_videoout2_h
//-------------------------------------------------------------------------------------------
