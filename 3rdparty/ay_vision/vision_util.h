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

// Access functions to an element of a cv::Vec, cv::Point, or cv::Point3_ with the same interface.
template<typename value_type, int N>
inline value_type& VecElem(cv::Vec<value_type,N> &v, int idx)  {return v(idx);}
template<typename value_type, int N>
inline const value_type& VecElem(const cv::Vec<value_type,N> &v, int idx)  {return v(idx);}
template<typename value_type>
inline value_type& VecElem(cv::Point_<value_type> &v, int idx)
{
  switch(idx)
  {
    case 0:  return v.x;
    case 1:  return v.y;
  }
  throw;
}
template<typename value_type>
inline const value_type& VecElem(const cv::Point_<value_type> &v, int idx)
{
  switch(idx)
  {
    case 0:  return v.x;
    case 1:  return v.y;
  }
  throw;
}
template<typename value_type>
inline value_type& VecElem(cv::Point3_<value_type> &v, int idx)
{
  switch(idx)
  {
    case 0:  return v.x;
    case 1:  return v.y;
    case 2:  return v.z;
  }
  throw;
}
template<typename value_type>
inline const value_type& VecElem(const cv::Point3_<value_type> &v, int idx)
{
  switch(idx)
  {
    case 0:  return v.x;
    case 1:  return v.y;
    case 2:  return v.z;
  }
  throw;
}
//-------------------------------------------------------------------------------------------

inline std::string GetPixelVal(const cv::Mat &m, int x, int y)
{
  std::stringstream ss;
  if(m.type()==CV_8UC1)        ss<<(int)m.at<unsigned char>(y,x);
  else if(m.type()==CV_8SC1)   ss<<(int)m.at<char>(y,x);
  else if(m.type()==CV_8UC3)   ss<<m.at<cv::Vec3b>(y,x);
  else if(m.type()==CV_16UC1)  ss<<m.at<unsigned short>(y,x);
  else if(m.type()==CV_16SC1)  ss<<m.at<short>(y,x);
  else if(m.type()==CV_32FC1)  ss<<m.at<float>(y,x);
  else if(m.type()==CV_32FC3)  ss<<m.at<cv::Vec3f>(y,x);
  else  ss<<"unknown type";
  return ss.str();
}
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


//-------------------------------------------------------------------------------------------
// 3D camera geometry utility.
//-------------------------------------------------------------------------------------------

// Get a projection matrix for resized image.
inline void GetProjMatForResizedImg(const cv::Mat &proj_mat, const double &resize_ratio, cv::Mat &proj_mat_s)
{
  proj_mat_s= resize_ratio*proj_mat;
  proj_mat_s.at<double>(2,2)= 1.0;
}
//-------------------------------------------------------------------------------------------

/*
Project 3D point to an image plane.
We assume a project matrix:
        [[Fx  0  Cx]
    P =  [ 0  Fy Cy]
         [ 0  0   1]]
Camera is at [0,0,0] and the image plane is z=1.
  A 3D point [xc,yc,zc]^T is projected onto an image plane [xp,yp] by:
    [u,v,w]^T= P * [xc,yc,zc]^T
    xp= u/w
    yp= v/w '''
*/
template<typename t_vec3d, typename t_vec2d>
inline void ProjectPointToImage(
    const t_vec3d &pt3d,
    const cv::Mat &proj_mat,
    t_vec2d &pt2d /*zmin=0.001*/)
{
  // if(VecElem(pt3d,2)<zmin)  return None;
  const double &Fx= proj_mat.at<double>(0,0);
  const double &Fy= proj_mat.at<double>(1,1);
  const double &Cx= proj_mat.at<double>(0,2);
  const double &Cy= proj_mat.at<double>(1,2);
  VecElem(pt2d,0)= (Fx*VecElem(pt3d,0)+Cx*VecElem(pt3d,2))/VecElem(pt3d,2);
  VecElem(pt2d,1)= (Fy*VecElem(pt3d,1)+Cy*VecElem(pt3d,2))/VecElem(pt3d,2);
}
//-------------------------------------------------------------------------------------------

// ProjectPointToImage for multiple input points pts3d (3D in camera frame).
template<typename t_vec3d, typename t_vec2d>
void ProjectPointToImageList(
    const std::vector<t_vec3d> &pts3d,
    const cv::Mat &proj_mat,
    std::vector<t_vec2d> &pts2d /*zmin=0.001*/)
{
  // if(any(pts3d[:,2]<zmin))  return None
  pts2d.resize(pts3d.size());
  typename std::vector<t_vec2d>::iterator itr_pt2d(pts2d.begin());
  for(typename std::vector<t_vec3d>::const_iterator itr_pt3d(pts3d.begin()),itr_pt3d_end(pts3d.end());
      itr_pt3d!=itr_pt3d_end; ++itr_pt3d,++itr_pt2d)
    ProjectPointToImage(*itr_pt3d, proj_mat, *itr_pt2d /*zmin*/);
}
//-------------------------------------------------------------------------------------------

inline bool IsValidDepth(int d, int d_max=10000)
{
  if(d>0 && d<=d_max)  return true;
  return false;
}
//-------------------------------------------------------------------------------------------

inline bool IsInvalidDepth(int d, int d_max=10000)
{
  if(d<=0 || d>d_max)  return true;
  return false;
}
//-------------------------------------------------------------------------------------------

// Transform a point on an image to a 3D point.
template<typename t_img_depth>
inline cv::Vec3f ImgPointTo3D(int u, int v, const t_img_depth &depth, const cv::Mat &proj_mat)
{
  const double &Fx= proj_mat.at<double>(0,0);
  const double &Fy= proj_mat.at<double>(1,1);
  const double &Cx= proj_mat.at<double>(0,2);
  const double &Cy= proj_mat.at<double>(1,2);
  const double d= depth * 0.001;
  return cv::Vec3f((u-Cx)/Fx*d, (v-Cy)/Fy*d, d);
}
//-------------------------------------------------------------------------------------------


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

// Since read and write for std::vector<T> is problematic in OpenCV 3.x, we define an alternative.
template<typename T>
void vec_read(const cv::FileNode& node, T &x, const T &x_default=T())
{
  cv::read(node,x,x_default);
}
template<typename T>
void vec_read(const cv::FileNode& node, std::vector<T> &x, const std::vector<T> &x_default=std::vector<T>())
{
  x.clear();
  for(cv::FileNodeIterator itr(node.begin()),itr_end(node.end()); itr!=itr_end; ++itr)
  {
    T y;
    vec_read(*itr,y);
    x.push_back(y);
  }
}
template<typename T>
void vec_write(cv::FileStorage &fs, const cv::String&, const T &x)
{
  cv::write(fs,"",x);
}
template<typename T>
void vec_write(cv::FileStorage &fs, const cv::String&, const std::vector<T> &x)
{
  fs<<"[";
  for(typename std::vector<T >::const_iterator itr(x.begin()),end(x.end());itr!=end;++itr)
  {
    vec_write(fs,"",*itr);
  }
  fs<<"]";
}
//-------------------------------------------------------------------------------------------
// Template to read from multiple YAML files.
// Assuming that ReadFromYAML(t_data &data, const std::string &file_name) is defined,
// and ReadFromYAML does not reset (clear) data when reading.
template<typename t_data>
void ReadFromYAML(t_data &data, const std::vector<std::string> &file_names)
{
  for(std::vector<std::string>::const_iterator itr(file_names.begin()),itr_end(file_names.end());
      itr!=itr_end; ++itr)
    ReadFromYAML(data, *itr);
}
//-------------------------------------------------------------------------------------------
// NOTE: kp_write and kp_read are different from write and read in OpenCV 3.4+ for KeyPoint.
void kp_write(cv::FileStorage &fs, const cv::String&, const cv::KeyPoint &x);
void kp_read(const cv::FileNode &data, cv::KeyPoint &x, const cv::KeyPoint &default_value=cv::KeyPoint());
void WriteToYAML(const std::vector<cv::KeyPoint> &keypoints, const std::string &file_name, cv::FileStorage *pfs=NULL);
void ReadFromYAML(std::vector<cv::KeyPoint> &keypoints, const std::string &file_name);
//-------------------------------------------------------------------------------------------

struct TCameraInfo
{
  std::string DevID;  // Video device ID (int) or Video/stream path (string)
  int Width, Height;  // Image size
  int FPS;  // Video frame rate
  std::string PixelFormat;  // Video codec (e.g. MJPG, YUYV)
  int CapWidth, CapHeight;  // Capture size (if zero, Width/Height is used; if not zero, captured image is resized to (Width,Height))
  cv::Rect CropRect;  // Crop the image after resizing to Width,Height. Disabled if one of them is negative.
  int HFlip;  // Whether flip image (horizontally), applied before NRotate90
  int NRotate90;  // Number of 90-deg rotations
  std::string Name;
  int Rectification;  // Whether rectify image or not
  double Alpha;     // Scaling factor
  cv::Mat K, D, R;  // Camera, distortion, and rectification matrices
  TCameraInfo()
      : DevID("0"),
        Width(0), Height(0), FPS(0),
        CapWidth(0), CapHeight(0),
        CropRect(-1,-1,-1,-1),
        HFlip(0), NRotate90(0),
        Rectification(0), Alpha(1.0) {}
};
//-------------------------------------------------------------------------------------------
bool CapOpen(TCameraInfo &info, cv::VideoCapture &cap);
bool CapWaitReopen(TCameraInfo &info, cv::VideoCapture &cap, int ms_wait=1000, int max_count=0, bool(*check_to_stop)(void)=NULL);
struct TCameraRectifier;
// Apply an image pre-processing to an image according to the camera info.
void Preprocess(cv::Mat &frame, const TCameraInfo &info, TCameraRectifier *pcam_rectifier=NULL);
void Print(const std::vector<TCameraInfo> &cam_info);
void WriteToYAML(const std::vector<TCameraInfo> &cam_info, const std::string &file_name, cv::FileStorage *pfs=NULL);
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
void WriteToYAML(const std::vector<TStereoInfo> &cam_info, const std::string &file_name, cv::FileStorage *pfs=NULL);
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
inline int cvRoundTmpl(const T &val)  {return cvRound(val);}
template<> inline int cvRoundTmpl<unsigned short>(const unsigned short &val)  {return cvRound((int)val);}
template<> inline int cvRoundTmpl<unsigned int>(const unsigned int &val)  {return cvRound((int)val);}
template<> inline int cvRoundTmpl<unsigned long>(const unsigned long &val)  {return cvRound((int)val);}
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
      return cvRoundTmpl((v-Min)/Step);
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
extern std::list<TExtendedTrackbarInfo<float> > ExtendedTrackbarInfo_float;
extern std::list<TExtendedTrackbarInfo<double> > ExtendedTrackbarInfo_double;
extern std::list<TExtendedTrackbarInfo<short> > ExtendedTrackbarInfo_short;
extern std::list<TExtendedTrackbarInfo<unsigned short> > ExtendedTrackbarInfo_unsigned_short;
extern std::list<TExtendedTrackbarInfo<int> > ExtendedTrackbarInfo_int;
extern std::list<TExtendedTrackbarInfo<unsigned int> > ExtendedTrackbarInfo_unsigned_int;
extern std::list<TExtendedTrackbarInfo<long> > ExtendedTrackbarInfo_long;
extern std::list<TExtendedTrackbarInfo<unsigned long> > ExtendedTrackbarInfo_unsigned_long;
extern std::list<TExtendedTrackbarInfo<bool> > ExtendedTrackbarInfo_bool;
extern std::list<TExtendedTrackbarInfo<std::string> > ExtendedTrackbarInfo_string;
template<typename T>
inline std::list<TExtendedTrackbarInfo<T> >& ExtendedTrackbarInfo();
template<>
inline std::list<TExtendedTrackbarInfo<float> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_float;}
template<>
inline std::list<TExtendedTrackbarInfo<double> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_double;}
template<>
inline std::list<TExtendedTrackbarInfo<short> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_short;}
template<>
inline std::list<TExtendedTrackbarInfo<unsigned short> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_unsigned_short;}
template<>
inline std::list<TExtendedTrackbarInfo<int> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_int;}
template<>
inline std::list<TExtendedTrackbarInfo<unsigned int> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_unsigned_int;}
template<>
inline std::list<TExtendedTrackbarInfo<long> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_long;}
template<>
inline std::list<TExtendedTrackbarInfo<unsigned long> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_unsigned_long;}
template<>
inline std::list<TExtendedTrackbarInfo<bool> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_bool;}
template<>
inline std::list<TExtendedTrackbarInfo<std::string> >& ExtendedTrackbarInfo()  {return ExtendedTrackbarInfo_string;}
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
// void write(cv::FileStorage &fs, const cv::String&, const cv::KeyPoint &x);
// void read(const cv::FileNode &data, cv::KeyPoint &x, const cv::KeyPoint &default_value=cv::KeyPoint());
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
