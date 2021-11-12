//-------------------------------------------------------------------------------------------
/*! \file    vision_util.cpp
    \brief   Basic vision utilities.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Oct.29, 2014
*/
//-------------------------------------------------------------------------------------------
#include "ay_vision/vision_util.h"
#include "ay_cpp/cpp_util.h"
#include "ay_cpp/sys_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <iomanip>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;

//-------------------------------------------------------------------------------------------
// struct TFPSEstimator
//-------------------------------------------------------------------------------------------
TFPSEstimator::TFPSEstimator(const double &init_fps, const double &alpha)
  :
    Alpha (alpha),
    FPS (init_fps),
    TimePrev (-1.0)
{
}
void TFPSEstimator::Step()
{
  if(TimePrev<0.0)
  {
    TimePrev= GetCurrentTime();
  }
  else
  {
    double new_fps= 1.0/(GetCurrentTime()-TimePrev);
    if(new_fps>FPS/20.0 && new_fps<FPS*20.0)  // Removing outliers (e.g. pause/resume)
      FPS= Alpha*new_fps + (1.0-Alpha)*FPS;
    TimePrev= GetCurrentTime();
  }
}
//-------------------------------------------------------------------------------------------


// Get median position of nonzero pixels
void GetMedian(const cv::Mat &src, int &x_med, int &y_med)
{
  assert(src.type()==CV_8UC1);
  int nonzero= cv::countNonZero(src);
  if(nonzero==0)  return;
  std::vector<int> arrayx(nonzero), arrayy(nonzero);
  int counter(0);
  for(int x(0); x<src.cols; ++x)
  {
    for(int y(0); y<src.rows; ++y)
    {
      if(src.at<unsigned char>(y,x) != 0)
      {
        arrayx[counter]= x;
        arrayy[counter]= y;
        ++counter;
      }
    }
  }
  std::sort(arrayx.begin(),arrayx.end());
  std::sort(arrayy.begin(),arrayy.end());
  x_med= arrayx[arrayx.size()/2];
  y_med= arrayy[arrayy.size()/2];
}
//-------------------------------------------------------------------------------------------

// Extract rows of src and store to dst (works for &dst==&src)
void ExtractRows(const cv::Mat &src, const std::vector<int> &idx, cv::Mat &dst)
{
  cv::Mat buf(src);
  dst.create(idx.size(),buf.cols,buf.type());
  int r(0);
  for(std::vector<int>::const_iterator itr(idx.begin()),itr_end(idx.end()); itr!=itr_end; ++itr,++r)
    buf.row(*itr).copyTo(dst.row(r));
}
//-------------------------------------------------------------------------------------------

// Find the largest contour and return info. bin_src should be a binary image.
// WARNING: bin_src is modified.
bool FindLargestContour(const cv::Mat &bin_src,
    double *area,
    cv::Point2d *center,
    cv::Rect *bound,
    std::vector<cv::Point> *contour)
{
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(bin_src,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  if(contours.size()==0)  return false;
  double a(0.0),a_max(0.0), i_max(0);
  for(int i(0),i_end(contours.size()); i<i_end; ++i)
  {
    a= cv::contourArea(contours[i],false);
    if(a>a_max)  {a_max= a;  i_max= i;}
  }
  std::vector<cv::Point> &cnt(contours[i_max]);
  if(area!=NULL)
    *area= a_max;
  if(center!=NULL)
  {
    cv::Moments mu= cv::moments(cnt);
    *center= cv::Point2d(mu.m10/mu.m00, mu.m01/mu.m00);
  }
  if(bound!=NULL)
    *bound= cv::boundingRect(cnt);
  if(contour!=NULL)
    *contour= cnt;
  return true;
}
//-------------------------------------------------------------------------------------------

// Rotate 90, 180, 270 degrees
void Rotate90N(const cv::Mat &src, cv::Mat &dst, int N)
{
  if(src.data!=dst.data)  src.copyTo(dst);
  for(int i((N%4+4)%4); i>0; --i)
  {
    cv::transpose(dst, dst);
    cv::flip(dst, dst, /*horizontal*/1);
  }
}
//-------------------------------------------------------------------------------------------

namespace ns_polygon_clip
{
#define TP cv::Point_<t_value>
#define ROW Row<t_value>
#define ADD Add<t_value>

template<typename t_value>
inline TP Row(const cv::Mat &points2d, int r)
{
  if(r<0)  r= points2d.rows+r;
  return TP(points2d.at<t_value>(r,0), points2d.at<t_value>(r,1));
}

template<typename t_value>
inline void Add(cv::Mat &points2d, const TP &p)
{
  cv::Mat v(p);
  v= v.t();
  points2d.push_back(v);
}

template<typename t_value>
inline int IsLeftOf(const TP &edge_1, const TP &edge_2, const TP &test)
{
  TP tmp1(edge_2.x - edge_1.x, edge_2.y - edge_1.y);
  TP tmp2(test.x - edge_2.x, test.y - edge_2.y);
  t_value x = (tmp1.x * tmp2.y) - (tmp1.y * tmp2.x);
  if(x < 0)  return 0;
  else if(x > 0)  return 1;
  else  return -1;  // Colinear points
}

template<typename t_value>
int IsClockwise(const cv::Mat &polygon)
{
  if(polygon.rows<3)  return -1;
  TP p0= ROW(polygon,0);
  TP p1= ROW(polygon,1);
  int isLeft(-1);
  for(int r(0),r_end(polygon.rows); r<r_end; ++r)
  {
    isLeft= IsLeftOf<t_value>(p0, p1, ROW(polygon,r));
    if(isLeft>=0)  // some of the points may be colinear.  That's ok as long as the overall is a polygon
      return isLeft==0 ? 1 : 0;
  }
  return -1;  // All the points in the polygon are colinear
}

template<typename t_value>
inline bool IsInside(const TP &cp1, const TP &cp2, const TP &p)
{
  return (cp2.x-cp1.x)*(p.y-cp1.y) > (cp2.y-cp1.y)*(p.x-cp1.x);
}

template<typename t_value>
inline TP ComputeIntersection(const TP &cp1, const TP &cp2, const TP &s, const TP &e)
{
  TP dc(cp1.x - cp2.x, cp1.y - cp2.y);
  TP dp(s.x - e.x, s.y - e.y);
  t_value n1= cp1.x * cp2.y - cp1.y * cp2.x;
  t_value n2= s.x * e.y - s.y * e.x;
  t_value n3= 1.0 / (dc.x * dp.y - dc.y * dp.x);
  return TP((n1*dp.x - n2*dc.x) * n3, (n1*dp.y - n2*dc.y) * n3);
}

template<typename t_value>
cv::Mat ClipPolygon_(const cv::Mat &polygon_subject_in, const cv::Mat &polygon_clip_in)
{
  cv::Mat polygon_empty(0,2,cv::DataType<t_value>::type);
  cv::Mat polygon_subject, polygon_clip;
  switch(IsClockwise<t_value>(polygon_subject_in))
  {
  case -1:
    std::cerr<<"polygon_subject: All the points are colinear"<<std::endl;
    return polygon_empty;
  case  0:  polygon_subject_in.copyTo(polygon_subject);  break;
  case +1:  cv::flip(polygon_subject_in, polygon_subject, 0);  break;
  }
  switch(IsClockwise<t_value>(polygon_clip_in))
  {
  case -1:
    std::cerr<<"polygon_clip: All the points are colinear"<<std::endl;
    return polygon_empty;
  case  0:  polygon_clip_in.copyTo(polygon_clip);  break;
  case +1:  cv::flip(polygon_clip_in, polygon_clip, 0);  break;
  }

  cv::Mat output_list= polygon_subject;
  TP cp1= ROW(polygon_clip,-1);

  for(int i_pc(0),i_pc_end(polygon_clip.rows); i_pc<i_pc_end; ++i_pc)
  {
    TP cp2= ROW(polygon_clip,i_pc);
    cv::Mat input_list= output_list;
    output_list= cv::Mat(0,2,cv::DataType<t_value>::type);
    if(input_list.rows==0)  return polygon_empty;
    TP s= ROW(input_list,-1);

    for(int i_in(0),i_in_end(input_list.rows); i_in<i_in_end; ++i_in)
    {
      TP e= ROW(input_list,i_in);
      if(IsInside<t_value>(cp1,cp2,e))
      {
        if(!IsInside<t_value>(cp1,cp2,s))
          ADD(output_list, ComputeIntersection<t_value>(cp1, cp2, s, e));
        ADD(output_list, e);
      }
      else if(IsInside<t_value>(cp1,cp2,s))
        ADD(output_list, ComputeIntersection<t_value>(cp1, cp2, s, e));
      s= e;
    }
    cp1= cp2;
  }
  return output_list;
}
#undef TP
#undef ROW
#undef ADD
}  // ns_polygon_clip
cv::Mat ClipPolygon(const cv::Mat &polygon_subject, const cv::Mat &polygon_clip)
{
  using namespace ns_polygon_clip;
  assert(polygon_subject.type()==polygon_clip.type());
  assert(polygon_subject.cols==2);
  assert(polygon_clip.cols==2);
  switch(polygon_subject.type())
  {
  case CV_32F:  return ClipPolygon_<float>(polygon_subject, polygon_clip);
  case CV_64F:  return ClipPolygon_<double>(polygon_subject, polygon_clip);
  case CV_16S:  return ClipPolygon_<short>(polygon_subject, polygon_clip);
  case CV_32S:  return ClipPolygon_<int>(polygon_subject, polygon_clip);
  }
  throw;
}
//-------------------------------------------------------------------------------------------

// Project points 3D onto a rectified image.
void ProjectPointsToRectifiedImg(const cv::Mat &points3d, const cv::Mat &P, cv::Mat &points2d)
{
  assert(points3d.type()==CV_32F);
  cv::Mat P2;
  P.convertTo(P2,points3d.type());
  // cv::Mat points3dh, points2dh;
  // cv::convertPointsToHomogeneous(points3d, points3dh);
  // points2dh= points3dh*P2.t();
  cv::Mat points2dh= points3d*P2(cv::Range(0,3),cv::Range(0,3)).t();
  cv::Mat p3= P2.col(3).t();
  for(int r(0),rows(points2dh.rows);r<rows;++r)
  {
    points2dh.row(r)+= p3;
    if(points2dh.at<float>(r,2)<0.0)  points2dh.at<float>(r,2)= 0.001;
    // if(points2dh.at<float>(r,2)<0.0)
    // {
      // points2dh.at<float>(r,2)= -points2dh.at<float>(r,2);
      // points2dh.at<float>(r,0)*= -1.0;
      // points2dh.at<float>(r,1)*= -1.0;
    // }
  }
  // float scale(0.0);
  // for(int r(0),rows(points2dh.rows);r<rows;++r)
  // {
    // points2dh.row(r)+= p3;
    // if(points2dh.at<float>(r,2)>0.0)
      // scale+= points2dh.at<float>(r,2);
    // else
    // {
      // scale+= 1.0;
      // points2dh.at<float>(r,0)*= -1.0;
      // points2dh.at<float>(r,1)*= -1.0;
    // }
  // }
  // scale/= (float)points2dh.rows;
  // for(int r(0),rows(points2dh.rows);r<rows;++r)
    // points2dh.at<float>(r,2)= scale;
  //*DBG*/std::cerr<<"..points2dh="<<points2dh<<std::endl;
  cv::convertPointsFromHomogeneous(points2dh, points2d);
  points2d= points2d.reshape(1);
  //*DBG*/std::cerr<<"..points2d="<<points2d<<std::endl;
  // cv::MatIterator_<float> itr= points2d.begin<float>();
  // cv::MatIterator_<float> itr_end= points2d.end<float>();
  // for(;itr!=itr_end;++itr)
  // {
    // if(*itr<-10000.0f)  *itr= -10000.0f;
    // else if(*itr>10000.0f)  *itr= 10000.0f;
  // }
  //*DBG*/std::cerr<<"..points2d="<<points2d<<std::endl;
}
//-------------------------------------------------------------------------------------------

void DrawCrossOnCenter(cv::Mat &img, int size, const cv::Scalar &col, int thickness)
{
  int hsize(size/2);
  cv::line(img, cv::Point(img.cols/2-hsize,img.rows/2), cv::Point(img.cols/2+hsize,img.rows/2), col, thickness);
  cv::line(img, cv::Point(img.cols/2,img.rows/2-hsize), cv::Point(img.cols/2,img.rows/2+hsize), col, thickness);
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

TEasyVideoOut::TEasyVideoOut(const double &init_fps)
  :
    file_prefix_ ("/tmp/video"),
    file_suffix_ (".avi"),
    img_size_ (0,0),
    fps_est_ (init_fps, /*alpha=*/0.05),
    is_recording_(false),
    stop_requested_(false)
{
}
//-------------------------------------------------------------------------------------------

// Start recording.
void TEasyVideoOut::Rec()
{
  if(!is_recording_)
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
    OpenVideoOut(writer_, file_name.c_str(), fps_est_.FPS, img_size_);
    is_recording_= true;
  }
}
//-------------------------------------------------------------------------------------------

// Stop recording.
void TEasyVideoOut::Stop()
{
  if(is_recording_)
    stop_requested_= true;
}
//-------------------------------------------------------------------------------------------

// Writing frame(during recording)/updating FPS,image size
void TEasyVideoOut::Step(const cv::Mat &frame)
{
  img_size_= cv::Size(frame.cols, frame.rows);

  // update fps
  fps_est_.Step();

  if(is_recording_)
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
  if(stop_requested_)
  {
    if(writer_.isOpened())
    {
      writer_.release();
      std::cerr<<"###Finished: video output"<<std::endl;
    }
    is_recording_= false;
    stop_requested_= false;
  }
}
//-------------------------------------------------------------------------------------------

/* Visualize a recording mark (red circle), effective only during recording.
    pos: position (0: left top (default), 1: left bottom, 2: right bottom, 3: right top)
*/
void TEasyVideoOut::VizRec(cv::Mat &frame, int pos, int rad, int margin) const
{
  if(is_recording_)
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

void kp_write(cv::FileStorage &fs, const cv::String&, const cv::KeyPoint &x)
{
  #define PROC_VAR(v)  fs<<#v<<x.v;
  fs<<"{";
  PROC_VAR(angle);
  PROC_VAR(class_id);
  PROC_VAR(octave);
  PROC_VAR(pt);
  PROC_VAR(response);
  PROC_VAR(size);
  fs<<"}";
  #undef PROC_VAR
}
//-------------------------------------------------------------------------------------------
void kp_read(const cv::FileNode &data, cv::KeyPoint &x, const cv::KeyPoint &default_value)
{
  #define PROC_VAR(v)  if(!data[#v].empty()) data[#v]>>x.v;
  PROC_VAR(angle);
  PROC_VAR(class_id);
  PROC_VAR(octave);
  PROC_VAR(pt);
  PROC_VAR(response);
  PROC_VAR(size);
  #undef PROC_VAR
}
//-------------------------------------------------------------------------------------------

void WriteToYAML(const std::vector<cv::KeyPoint> &keypoints, const std::string &file_name)
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  // fs<<"KeyPoints"<<keypoints;
  fs<<"KeyPoints"<<"[";
  for(std::vector<cv::KeyPoint>::const_iterator itr(keypoints.begin()),itr_end(keypoints.end()); itr!=itr_end; ++itr)
  {
    kp_write(fs,"",*itr);
  }
  fs<<"]";
  fs.release();
}
//-------------------------------------------------------------------------------------------

void ReadFromYAML(std::vector<cv::KeyPoint> &keypoints, const std::string &file_name)
{
  keypoints.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data= fs["KeyPoints"];
  // data>>keypoints;
  for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
  {
    cv::KeyPoint kp;
    kp_read(*itr,kp,cv::KeyPoint());
    keypoints.push_back(kp);
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------


// Open a video capture cap based on the camera information info.
// If info.CapWidth or info.CapWidth is zero, info.Width or info.Height is assigned.
// Return if opened.
bool CapOpen(TCameraInfo &info, cv::VideoCapture &cap)
{
  const std::string &fourcc(info.PixelFormat);
  if(IsInt(info.DevID))
  {
    int dev_id= ToInt(info.DevID);
    cap.open(dev_id); cap.release();  // Trick of robust open
    cap.open(dev_id);
  }
  else
  {
    cap.open(info.DevID);
  }
  if(!cap.isOpened())
  {
    std::cerr<<"Failed to open camera: "<<info.DevID<<std::endl;
    return false;
  }
  if(fourcc.size()>0)  cap.set(CV_CAP_PROP_FOURCC,CV_FOURCC(fourcc[0],fourcc[1],fourcc[2],fourcc[3]));
  if(info.CapWidth==0)  info.CapWidth= info.Width;
  if(info.CapHeight==0)  info.CapHeight= info.Height;
  cap.set(CV_CAP_PROP_FRAME_WIDTH, info.CapWidth);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, info.CapHeight);
  return true;
}
//-------------------------------------------------------------------------------------------

// Use when there is a trouble in capturing to wait for recovering the reconnection.
bool CapWaitReopen(TCameraInfo &info, cv::VideoCapture &cap, int ms_wait, int max_count, bool(*check_to_stop)(void))
{
  cap.release();
  for(int i(0); ((max_count>0) ? (i<max_count) : true); ++i)
  {
    if(check_to_stop && check_to_stop())  break;
    char c(cv::waitKey(ms_wait));
    if(c=='\x1b'||c=='q')  break;
    cap.release();
    if(CapOpen(info, cap))  break;
  }
  return cap.isOpened();
}
//-------------------------------------------------------------------------------------------

void Print(const std::vector<TCameraInfo> &cam_info)
{
  int i(0);
  for(std::vector<TCameraInfo>::const_iterator itr(cam_info.begin()),itr_end(cam_info.end()); itr!=itr_end; ++itr,++i)
  {
    std::cout<<"No. "<<i<<std::endl;
    #define PROC_VAR(x)  std::cout<<"  "#x": "<<itr->x<<std::endl;
    PROC_VAR(DevID       );
    PROC_VAR(Width       );
    PROC_VAR(Height      );
    PROC_VAR(PixelFormat );
    PROC_VAR(HFlip       );
    PROC_VAR(NRotate90   );
    PROC_VAR(CapWidth    );
    PROC_VAR(CapHeight   );
    PROC_VAR(Name        );
    PROC_VAR(Rectification);
    PROC_VAR(Alpha        );
    PROC_VAR(K            );
    PROC_VAR(D            );
    PROC_VAR(R            );
    #undef PROC_VAR
  }
}
//-------------------------------------------------------------------------------------------

void WriteToYAML(const std::vector<TCameraInfo> &cam_info, const std::string &file_name)
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  fs<<"CameraInfo"<<"[";
  for(std::vector<TCameraInfo>::const_iterator itr(cam_info.begin()),itr_end(cam_info.end()); itr!=itr_end; ++itr)
  {
    fs<<(*itr);
  }
  fs<<"]";
  fs.release();
}
//-------------------------------------------------------------------------------------------

void ReadFromYAML(std::vector<TCameraInfo> &cam_info, const std::string &file_name)
{
  cam_info.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data= fs["CameraInfo"];
  for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
  {
    TCameraInfo cf;
    (*itr)>>cf;
    cam_info.push_back(cf);
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------


void Print(const std::vector<TStereoInfo> &stereo_info)
{
  int i(0);
  for(std::vector<TStereoInfo>::const_iterator itr(stereo_info.begin()),itr_end(stereo_info.end()); itr!=itr_end; ++itr,++i)
  {
    std::cout<<"No. "<<i<<std::endl;
    #define PROC_VAR(x)  std::cout<<"  "#x": "<<itr->x<<std::endl;
    PROC_VAR(Name        );
    PROC_VAR(CamL        );
    PROC_VAR(CamR        );
    PROC_VAR(Width       );
    PROC_VAR(Height      );
    PROC_VAR(StereoParam );
    PROC_VAR(StereoConfig);
    #undef PROC_VAR
  }
}
//-------------------------------------------------------------------------------------------

void WriteToYAML(const std::vector<TStereoInfo> &stereo_info, const std::string &file_name)
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  fs<<"StereoInfo"<<"[";
  for(std::vector<TStereoInfo>::const_iterator itr(stereo_info.begin()),itr_end(stereo_info.end()); itr!=itr_end; ++itr)
  {
    #define PROC_VAR(x)  fs<<#x<<itr->x;
    fs<<"{";
    PROC_VAR(Name        );
    PROC_VAR(CamL        );
    PROC_VAR(CamR        );
    PROC_VAR(Width       );
    PROC_VAR(Height      );
    PROC_VAR(StereoParam );
    PROC_VAR(StereoConfig);
    fs<<"}";
    #undef PROC_VAR
  }
  fs<<"]";
  fs.release();
}
//-------------------------------------------------------------------------------------------

void ReadFromYAML(std::vector<TStereoInfo> &stereo_info, const std::string &file_name)
{
  stereo_info.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data= fs["StereoInfo"];
  for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
  {
    TStereoInfo cf;
    #define PROC_VAR(x)  if(!(*itr)[#x].empty())  (*itr)[#x]>>cf.x;
    PROC_VAR(Name        );
    PROC_VAR(CamL        );
    PROC_VAR(CamR        );
    PROC_VAR(Width       );
    PROC_VAR(Height      );
    PROC_VAR(StereoParam );
    PROC_VAR(StereoConfig);
    #undef PROC_VAR
    stereo_info.push_back(cf);
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// struct TCameraRectifier
//-------------------------------------------------------------------------------------------
void TCameraRectifier::Setup(const cv::Mat &K, const cv::Mat &D, const cv::Mat &R, const cv::Size &size_in, const double &alpha, const cv::Size &size_out)
{
  cv::Mat P= cv::getOptimalNewCameraMatrix(K, D, size_in, alpha, size_out);
  cv::initUndistortRectifyMap(K, D, R, P, size_out, CV_16SC2, map1_, map2_);
}
//-------------------------------------------------------------------------------------------

void TCameraRectifier::Rectify(cv::Mat &frame, const cv::Scalar& border)
{
  cv::Mat framer;
  cv::remap(frame, framer, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, border);
  frame= framer;
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
namespace cv
{

// void write(cv::FileStorage &fs, const cv::String&, const cv::Point2f &x)
// {
//   #define PROC_VAR(v)  fs<<#v<<x.v;
//   fs<<"{";
//   PROC_VAR(x);
//   PROC_VAR(y);
//   fs<<"}";
//   #undef PROC_VAR
// }
// //-------------------------------------------------------------------------------------------
// void read(const cv::FileNode &data, cv::Point2f &x, const cv::Point2f &default_value)
// {
//   #define PROC_VAR(v)  if(!data[#v].empty()) data[#v]>>x.v;
//   PROC_VAR(x);
//   PROC_VAR(y);
//   #undef PROC_VAR
// }
// //-------------------------------------------------------------------------------------------
// void write(cv::FileStorage &fs, const cv::String&, const cv::KeyPoint &x)
// {
//   #define PROC_VAR(v)  fs<<#v<<x.v;
//   fs<<"{";
//   PROC_VAR(angle);
//   PROC_VAR(class_id);
//   PROC_VAR(octave);
//   PROC_VAR(pt);
//   PROC_VAR(response);
//   PROC_VAR(size);
//   fs<<"}";
//   #undef PROC_VAR
// }
// //-------------------------------------------------------------------------------------------
// void read(const cv::FileNode &data, cv::KeyPoint &x, const cv::KeyPoint &default_value)
// {
//   #define PROC_VAR(v)  if(!data[#v].empty()) data[#v]>>x.v;
//   PROC_VAR(angle);
//   PROC_VAR(class_id);
//   PROC_VAR(octave);
//   PROC_VAR(pt);
//   PROC_VAR(response);
//   PROC_VAR(size);
//   #undef PROC_VAR
// }
// //-------------------------------------------------------------------------------------------

// void write(cv::FileStorage &fs, const cv::String&, const cv::SimpleBlobDetector::Params &x)
// {
  // x.write(fs);
// }
// //-------------------------------------------------------------------------------------------
// void read(const cv::FileNode &data, cv::SimpleBlobDetector::Params &x, const cv::SimpleBlobDetector::Params &default_value)
// {
  // x.read(data);
// }
// //-------------------------------------------------------------------------------------------

void write(cv::FileStorage &fs, const cv::String&, const trick::TCameraInfo &x)
{
  #define PROC_VAR(v)  fs<<#v<<x.v;
  fs<<"{";
  PROC_VAR(DevID       );
  PROC_VAR(Width       );
  PROC_VAR(Height      );
  PROC_VAR(PixelFormat );
  PROC_VAR(HFlip       );
  PROC_VAR(NRotate90   );
  PROC_VAR(CapWidth    );
  PROC_VAR(CapHeight   );
  PROC_VAR(Name        );
  PROC_VAR(Rectification);
  PROC_VAR(Alpha        );
  PROC_VAR(K            );
  PROC_VAR(D            );
  PROC_VAR(R            );
  fs<<"}";
  #undef PROC_VAR
}
//-------------------------------------------------------------------------------------------
void read(const cv::FileNode &data, trick::TCameraInfo &x, const trick::TCameraInfo &default_value)
{
  #define PROC_VAR(v)  if(!data[#v].empty())  data[#v]>>x.v;
  PROC_VAR(DevID       );
  PROC_VAR(Width       );
  PROC_VAR(Height      );
  PROC_VAR(PixelFormat );
  PROC_VAR(HFlip       );
  PROC_VAR(NRotate90   );
  PROC_VAR(CapWidth    );
  PROC_VAR(CapHeight   );
  PROC_VAR(Name        );
  PROC_VAR(Rectification);
  PROC_VAR(Alpha        );
  PROC_VAR(K            );
  PROC_VAR(D            );
  PROC_VAR(R            );
  #undef PROC_VAR
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // namespace cv
//-------------------------------------------------------------------------------------------
