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
#include <vector>
#include <list>
#include <iostream>
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
void ExtractRows(const cv::Mat &src, const std::vector<int> &idx, cv::Mat &dst);
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

void DrawCrossOnCenter(cv::Mat &img, int size, const cv::Scalar &col, int thickness=1);
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
      if(!is_recording_)  Rec();
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

  bool IsRecording() const {return is_recording_;}
  const double& FPS() const {return fps_est_.FPS;}

  void SetfilePrefix(const std::string &v)  {file_prefix_= v;}
  void SetfileSuffix(const std::string &v)  {file_suffix_= v;}

private:
  std::string file_prefix_, file_suffix_;
  cv::VideoWriter writer_;
  cv::Size img_size_;
  TFPSEstimator fps_est_;
  bool is_recording_, stop_requested_;
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
  int HFlip;  // Whether flip image (horizontally), applied before NRotate90
  int NRotate90;  // Number of 90-deg rotations
  int CapWidth, CapHeight;  // Capture size (if zero, Width/Height is used; if not zero, captured image is resized to (Width,Height))
  std::string Name;
  int Rectification;  // Whether rectify image or not
  double Alpha;     // Scaling factor
  cv::Mat K, D, R;  // Camera, distortion, and rectification matrices
  TCameraInfo()
      : HFlip(0), NRotate90(0), CapWidth(0), CapHeight(0), Rectification(0), Alpha(1.0) {}
};
//-------------------------------------------------------------------------------------------
bool CapOpen(TCameraInfo &info, cv::VideoCapture &cap);
bool CapWaitReopen(TCameraInfo &info, cv::VideoCapture &cap, int ms_wait=1000, int max_count=0, bool(*check_to_stop)(void)=NULL);
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
// Extended trackbar class where trackbars can be defined with min/max/step for int/float/double/bool,
// and trackbars can be defined with std::vector<std::string> for std::string.
//-------------------------------------------------------------------------------------------
template<typename T>
struct TExtendedTrackbarInfo;
template<typename T>
struct TExtendedTrackbarUtil
{
  typedef T TTrackValue;
  static T Convert(const TExtendedTrackbarInfo<T> &info, const TTrackValue &v)
    {
      return v;
    }
  static TTrackValue Invert(const TExtendedTrackbarInfo<T> &info, const T &v)
    {
      return v;
    }
};
template<typename T>
struct TExtendedTrackbarInfo
{
  typedef typename TExtendedTrackbarUtil<T>::TTrackValue TTrackValue;
  const std::string Name, WinName;
  int Position;
  int IntMax;
  T &Value;
  TTrackValue Min;
  TTrackValue Max;
  TTrackValue Step;
  typedef void (*TCallback)(const TExtendedTrackbarInfo<T>&, void*);
  TCallback OnUpdate;
  void *Reference;
  void *UserData;
  TExtendedTrackbarInfo(const std::string &name, const std::string &winname, T &value, const TTrackValue &min, const TTrackValue &max, const TTrackValue &step, TCallback on_update, void *user_data, void *reference)
    : Name(name), WinName(winname), Value(value), OnUpdate(NULL), Reference(NULL), UserData(NULL)
    {
      Min= min;
      Max= max;
      Step= step;
      IntMax= ToInt(Max);
      if(on_update)  OnUpdate= on_update;
      if(user_data)  UserData= user_data;
      if(reference)  Reference= reference;
      Position= ToInt(TExtendedTrackbarUtil<T>::Invert(*this, value));
    }
  T ToValue(int p) const
    {
      if(p>IntMax)  p= IntMax;
      if(p<0)  p= 0;
      return TExtendedTrackbarUtil<T>::Convert(*this, Min + Step*static_cast<TTrackValue>(p));
    }
  T ToValue() const
    {
      return ToValue(Position);
    }
  int ToInt(TTrackValue v) const
    {
      if(v>Max)  v= Max;
      if(v<Min)  v= Min;
      return cvRound((v-Min)/Step);
    }
  void Update()
    {
      Value= ToValue();
      if(OnUpdate)  OnUpdate(*this, UserData);
    }
};
template<>
struct TExtendedTrackbarUtil<std::string>
{
  typedef int TTrackValue;
  static std::string Convert(const TExtendedTrackbarInfo<std::string> &info, const TTrackValue &v)
    {
      return (*reinterpret_cast<const std::vector<std::string>*>(info.Reference))[v];
    }
  static TTrackValue Invert(const TExtendedTrackbarInfo<std::string> &info, const std::string &v)
    {
      const std::vector<std::string> &ref(*reinterpret_cast<const std::vector<std::string>*>(info.Reference));
      std::vector<std::string>::const_iterator itr= std::find(ref.begin(),ref.end(),v);
      if(itr==ref.end())  return -1;
      return std::distance(ref.begin(), itr);
    }
};
std::list<TExtendedTrackbarInfo<float> > ExtendedTrackbarInfo_float;
std::list<TExtendedTrackbarInfo<double> > ExtendedTrackbarInfo_double;
std::list<TExtendedTrackbarInfo<int> > ExtendedTrackbarInfo_int;
std::list<TExtendedTrackbarInfo<bool> > ExtendedTrackbarInfo_bool;
std::list<TExtendedTrackbarInfo<std::string> > ExtendedTrackbarInfo_string;
template<typename T>
std::list<TExtendedTrackbarInfo<T> >& ExtendedTrackbarInfo();
template<>
std::list<TExtendedTrackbarInfo<float> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_float;}
template<>
std::list<TExtendedTrackbarInfo<double> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_double;}
template<>
std::list<TExtendedTrackbarInfo<int> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_int;}
template<>
std::list<TExtendedTrackbarInfo<bool> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_bool;}
template<>
std::list<TExtendedTrackbarInfo<std::string> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_string;}
template<typename T>
void ExtendedTrackbarOnChange(int,void *pi)
{
  TExtendedTrackbarInfo<T> &info(*reinterpret_cast<TExtendedTrackbarInfo<T>*>(pi));
  info.Update();
}
//-------------------------------------------------------------------------------------------
template<typename T>
int CreateTrackbarHelper(const std::string& trackbarname, const std::string& winname, T *value, const typename TExtendedTrackbarUtil<T>::TTrackValue &min, const typename TExtendedTrackbarUtil<T>::TTrackValue &max, const typename TExtendedTrackbarUtil<T>::TTrackValue &step, typename TExtendedTrackbarInfo<T>::TCallback on_track=NULL, void *user_data=NULL, void *reference=NULL)
{
  for(typename std::list<TExtendedTrackbarInfo<T> >::iterator itr(ExtendedTrackbarInfo<T>().begin()),itr_end(ExtendedTrackbarInfo<T>().end()); itr!=itr_end; ++itr)
  {
    if(itr->Name==trackbarname && itr->WinName==winname)
    {
      ExtendedTrackbarInfo<T>().erase(itr);
      break;
    }
  }
  ExtendedTrackbarInfo<T>().push_back(TExtendedTrackbarInfo<T>(trackbarname, winname, *value, min, max, step, on_track, user_data, reference));
  TExtendedTrackbarInfo<T> &pi(ExtendedTrackbarInfo<T>().back());
  return cv::createTrackbar(trackbarname, winname, &pi.Position, pi.IntMax, ExtendedTrackbarOnChange<T>, &pi);
}
template<typename T>
int CreateTrackbar(const std::string& trackbarname, const std::string& winname, T *value, const T &min, const T &max, const T &step, typename TExtendedTrackbarInfo<T>::TCallback on_track=NULL, void *user_data=NULL)
{
  return CreateTrackbarHelper<T>(trackbarname, winname, value, min, max, step, on_track, user_data);
}
template<typename T>
int CreateTrackbar(const std::string& trackbarname, const std::string& winname, T *value, TExtendedTrackbarInfo<bool>::TCallback on_track=NULL, void *user_data=NULL)
{
  return CreateTrackbarHelper<T>(trackbarname, winname, value, 0, 1, 1, on_track, user_data);
}
template<typename T>
int CreateTrackbar(const std::string& trackbarname, const std::string& winname, T *value, std::vector<std::string> &str_list, typename TExtendedTrackbarInfo<T>::TCallback on_track=NULL, void *user_data=NULL)
{
  return CreateTrackbarHelper<T>(trackbarname, winname, value, 0, str_list.size()-1, 1, on_track, user_data, &str_list);
}
//-------------------------------------------------------------------------------------------
template<typename T>
void TrackbarPrintOnTrack(const TExtendedTrackbarInfo<T> &info, void*)
{
  std::cerr<<info.Name<<"= "<<info.Value<<std::endl;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
namespace cv
{
//-------------------------------------------------------------------------------------------
// void write(cv::FileStorage &fs, const cv::String&, const cv::Point2f &x);
// void read(const cv::FileNode &data, cv::Point2f &x, const cv::Point2f &default_value=cv::Point2f());
void write(cv::FileStorage &fs, const cv::String&, const cv::KeyPoint &x);
void read(const cv::FileNode &data, cv::KeyPoint &x, const cv::KeyPoint &default_value=cv::KeyPoint());
// void write(cv::FileStorage &fs, const cv::String&, const cv::SimpleBlobDetector::Params &x);
// void read(const cv::FileNode &data, cv::SimpleBlobDetector::Params &x, const cv::SimpleBlobDetector::Params &default_value=cv::SimpleBlobDetector::Params());

void write(cv::FileStorage &fs, const cv::String&, const trick::TCameraInfo &x);
void read(const cv::FileNode &data, trick::TCameraInfo &x, const trick::TCameraInfo &default_value=trick::TCameraInfo());

// For saving vector of vector.
// template<typename T>
// void write(cv::FileStorage &fs, const cv::String&, const std::vector<std::vector<T> > &x)
// {
//   fs<<"[";
//   for(typename std::vector<std::vector<T> >::const_iterator itr(x.begin()),end(x.end());itr!=end;++itr)
//   {
//     fs<<*itr;
//   }
//   fs<<"]";
// }

// Define a new bool reader in order to accept "true/false"-like values.
inline void read_bool(const cv::FileNode &node, bool &value, const bool &default_value)
{
  std::string s(static_cast<std::string>(node));
  if(s=="y"||s=="Y"||s=="yes"||s=="Yes"||s=="YES"||s=="true"||s=="True"||s=="TRUE"||s=="on"||s=="On"||s=="ON")
    {value=true; return;}
  if(s=="n"||s=="N"||s=="no"||s=="No"||s=="NO"||s=="false"||s=="False"||s=="FALSE"||s=="off"||s=="Off"||s=="OFF")
    {value=false; return;}
  value= static_cast<int>(node);
}
// Specialize cv::operator>> for bool.
template<> inline void operator >> (const cv::FileNode& n, bool& value)
{
  read_bool(n, value, false);
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // namespace cv
//-------------------------------------------------------------------------------------------
#endif // vision_util_h
//-------------------------------------------------------------------------------------------
