//-------------------------------------------------------------------------------------------
/*! \file    cap_open.h
    \brief   Opening a camera device or stream.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.08, 2017
*/
//-------------------------------------------------------------------------------------------
#ifndef cap_open_h
#define cap_open_h
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cstdlib>  // strtol, atol
//-------------------------------------------------------------------------------------------
// namespace loco_rabbits
// {
//-------------------------------------------------------------------------------------------

inline bool IsInt(const std::string &s)
{
  if(s.empty() || std::isspace(s[0]))  return false;
  char *p;
  strtol(s.c_str(), &p, 10);
  return (*p == 0);
}
//-------------------------------------------------------------------------------------------

inline int ToInt(const std::string &s)
{
  return atol(s.c_str());
}
//-------------------------------------------------------------------------------------------

cv::VideoCapture CapOpen(const std::string &src, int width=0, int height=0, const std::string &fourcc="")
{
  cv::VideoCapture cap;
  if(IsInt(src))
  {
    int dev_id= ToInt(src);
    cap.open(dev_id); cap.release();  // Trick of robust open
    cap.open(dev_id);
  }
  else
  {
    cap.open(src);
  }
  if(!cap.isOpened())
  {
    std::cerr<<"Failed to open camera: "<<src<<std::endl;
    return cap;
  }
  std::cerr<<"Opened camera: "<<src<<std::endl;
  if(fourcc.size()>0)  cap.set(CV_CAP_PROP_FOURCC,CV_FOURCC(fourcc[0],fourcc[1],fourcc[2],fourcc[3]));
  if(width!=0)   cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
  if(height!=0)  cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
  return cap;
}
//-------------------------------------------------------------------------------------------

struct TCapture
{
  cv::VideoCapture C;
  std::string DevID;  // Video device ID (int) or Video/stream path (string
  int CapWidth;
  int CapHeight;
  std::string PixelFormat;

  bool Open(const std::string &src, int width=0, int height=0, const std::string &fourcc="")
    {
      DevID= src;
      CapWidth= width;
      CapHeight= height;
      PixelFormat= fourcc;
      C= CapOpen(DevID, CapWidth, CapHeight, PixelFormat);
      return C.isOpened();
    }
  bool Reopen()
    {
      C.release();
      C= CapOpen(DevID, CapWidth, CapHeight, PixelFormat);
      return C.isOpened();
    }
  bool WaitReopen(int ms_wait=1000, int max_count=0)
    {
      C.release();
      for(int i(0); ((max_count>0) ? (i<max_count) : true); ++i)
      {
        char c(cv::waitKey(ms_wait));
        if(c=='\x1b'||c=='q')  break;
        if(Reopen())  break;
      }
      return C.isOpened();
    }
  bool Read(cv::Mat &frame)
    {
        C >> frame;
        if(!frame.empty())  return true;
        return false;
    }
  TCapture& operator>>(cv::Mat &frame)
    {
      Read(frame);
      return *this;
    }
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// }  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // cap_open_h
//-------------------------------------------------------------------------------------------
