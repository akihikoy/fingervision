//-------------------------------------------------------------------------------------------
/*! \file    vision_util.h
    \brief   Basic vision utilities.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.29, 2014
*/
//-------------------------------------------------------------------------------------------
#ifndef vision_util_h
#define vision_util_h
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
//-------------------------------------------------------------------------------------------
namespace cv
{
void write(cv::FileStorage &fs, const std::string&, const cv::Point2f &x);
void read(const cv::FileNode &data, cv::Point2f &x, const cv::Point2f &default_value=cv::Point2f());
void write(cv::FileStorage &fs, const std::string&, const cv::KeyPoint &x);
void read(const cv::FileNode &data, cv::KeyPoint &x, const cv::KeyPoint &default_value=cv::KeyPoint());
// void write(cv::FileStorage &fs, const std::string&, const cv::SimpleBlobDetector::Params &x);
// void read(const cv::FileNode &data, cv::SimpleBlobDetector::Params &x, const cv::SimpleBlobDetector::Params &default_value=cv::SimpleBlobDetector::Params());

// For saving vector of vector.
template<typename T>
void write(cv::FileStorage &fs, const std::string&, const std::vector<std::vector<T> > &x)
{
  fs<<"[";
  for(typename std::vector<std::vector<T> >::const_iterator itr(x.begin()),end(x.end());itr!=end;++itr)
  {
    fs<<*itr;
  }
  fs<<"]";
}
//-------------------------------------------------------------------------------------------

}  // namespace cv
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

inline float Dist(const cv::Point2f &p, const cv::Point2f &q)
{
  cv::Point2f d= p-q;
  return cv::sqrt(d.x*d.x + d.y*d.y);
}
//-------------------------------------------------------------------------------------------

// cv::absdiff with mask: res=abs(a-b)
inline void absdiff(const cv::Mat &a, const cv::Mat &b, cv::Mat &res, cv::InputArray mask=cv::noArray(), int dtype=-1)
{
  cv::Mat aa,bb,cc;
  a.convertTo(aa, CV_16SC3);
  b.convertTo(bb, CV_16SC3);
  cv::subtract(aa, bb, cc, mask, dtype);
  cc= cv::abs(cc);
  cc.convertTo(res, a.type());
}
//-------------------------------------------------------------------------------------------

struct TFPSEstimator
{
  double Alpha;
  double FPS;
  double TimePrev;
  TFPSEstimator(const double &init_fps=10.0, const double &alpha=0.05);
  void Step();
};
//-------------------------------------------------------------------------------------------

// Get median position of nonzero pixels
void GetMedian(const cv::Mat &src, int &x_med, int &y_med);
//-------------------------------------------------------------------------------------------

// Extract rows of src and store to dst (works for &dst==&src)
void ExtractRows(const cv::Mat &src, const cv::vector<int> &idx, cv::Mat &dst);
//-------------------------------------------------------------------------------------------

// Find the largest contour and return info. bin_src should be a binary image.
bool FindLargestContour(const cv::Mat &bin_src,
    double *area=NULL,
    cv::Point2d *center=NULL,
    cv::Rect *bound=NULL,
    std::vector<cv::Point> *contour=NULL);
//-------------------------------------------------------------------------------------------

// Rotate 90, 180, 270 degrees
void Rotate90N(const cv::Mat &src, cv::Mat &dst, int N);
//-------------------------------------------------------------------------------------------

cv::Mat ClipPolygon(const cv::Mat &polygon_subject, const cv::Mat &polygon_clip);
//-------------------------------------------------------------------------------------------

// Project points 3D onto a rectified image.
void ProjectPointsToRectifiedImg(const cv::Mat &points3d, const cv::Mat &P, cv::Mat &points2d);
//-------------------------------------------------------------------------------------------

inline void DrawCrossOnCenter(cv::Mat &img, int size, const cv::Scalar &col, int thickness=1)
{
  int hsize(size/2);
  cv::line(img, cv::Point(img.cols/2-hsize,img.rows/2), cv::Point(img.cols/2+hsize,img.rows/2), col, thickness);
  cv::line(img, cv::Point(img.cols/2,img.rows/2-hsize), cv::Point(img.cols/2,img.rows/2+hsize), col, thickness);
}
//-------------------------------------------------------------------------------------------

// Convert a mask to a color image.
inline cv::Mat ColorMask(cv::Mat mask, const cv::Scalar &col)
{
  cv::Mat cmask(mask.size(),CV_8UC3,cv::Scalar(0,0,0));
  cmask.setTo(col,mask);
  return cmask;
}
//-------------------------------------------------------------------------------------------

bool OpenVideoOut(cv::VideoWriter &vout, const char *file_name, int fps, const cv::Size &size);
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
  TEasyVideoOut(const double &init_fps=10.0);

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
  const double& FPS() const {return fps_est_.FPS;}

  void SetfilePrefix(const std::string &v)  {file_prefix_= v;}
  void SetfileSuffix(const std::string &v)  {file_suffix_= v;}

private:
  std::string file_prefix_, file_suffix_;
  cv::VideoWriter writer_;
  cv::Size img_size_;
  TFPSEstimator fps_est_;
};
//-------------------------------------------------------------------------------------------

void WriteToYAML(const std::vector<cv::KeyPoint> &keypoints, const std::string &file_name);
void ReadFromYAML(std::vector<cv::KeyPoint> &keypoints, const std::string &file_name);
//-------------------------------------------------------------------------------------------

struct TCameraInfo
{
  std::string DevID;  // Video device ID (int) or Video/stream path (string)
  int Width, Height;  // Image size
  std::string PixelFormat;
  int NRotate90;
  int CapWidth, CapHeight;  // Capture size (if zero, Width/Height is used; if not zero, captured image is resized to (Width,Height))
  std::string Name;
  int Rectification;  // Whether rectify image or not
  double Alpha;     // Scaling factor
  cv::Mat K, D, R;  // Camera, distortion, and rectification matrices
  TCameraInfo()
      : NRotate90(0), CapWidth(0), CapHeight(0), Rectification(0), Alpha(1.0) {}
};
//-------------------------------------------------------------------------------------------
bool CapOpen(TCameraInfo &info, cv::VideoCapture &cap);
bool CapWaitReopen(TCameraInfo &info, cv::VideoCapture &cap, int ms_wait=1000, int max_count=0);
void Print(const std::vector<TCameraInfo> &cam_info);
void WriteToYAML(const std::vector<TCameraInfo> &cam_info, const std::string &file_name);
void ReadFromYAML(std::vector<TCameraInfo> &cam_info, const std::string &file_name);
//-------------------------------------------------------------------------------------------

struct TStereoInfo
{
  std::string Name;
  int CamL, CamR;
  int Width, Height;
  std::string StereoParam;  // Stereo camera parameters
  std::string StereoConfig;  // Stereo algorithm configurations
};
//-------------------------------------------------------------------------------------------
void Print(const std::vector<TStereoInfo> &cam_info);
void WriteToYAML(const std::vector<TStereoInfo> &cam_info, const std::string &file_name);
void ReadFromYAML(std::vector<TStereoInfo> &cam_info, const std::string &file_name);
//-------------------------------------------------------------------------------------------

struct TCameraRectifier
{
  cv::Mat map1_, map2_;
  void Setup(const cv::Mat &K, const cv::Mat &D, const cv::Mat &R, const cv::Size &size_in, const double &alpha, const cv::Size &size_out);
  void Rectify(cv::Mat &frame, const cv::Scalar& border=cv::Scalar());
};
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // vision_util_h
//-------------------------------------------------------------------------------------------
