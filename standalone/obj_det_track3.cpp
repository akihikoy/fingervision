//-------------------------------------------------------------------------------------------
/*! \file    obj_det_track3.cpp
    \brief   Object detection and tracking
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.17, 2017

Based on obj_det_track1.cpp
Provides a good interface for integration.

g++ -I -Wall obj_det_track3.cpp -o obj_det_track3.out -lopencv_core -lopencv_video -lopencv_imgproc -lopencv_highgui
*/
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "cv2-videoout2.h"
#include "rotate90n.h"
#include "cap_open.h"
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

struct TObjDetTrackBSPParams
{
  // Resize image for speed up
  int Width, Height;

  // Remove black background
  int ThreshBlkH;
  int ThreshBlkS;
  int ThreshBlkV;

  // Parameters of BackgroundSubtractorMOG2
  float BS_History;
  float BS_VarThreshold;
  bool BS_DetectShadows;

  // Background subtraction and post process
  int NErode1;  // Size of erode applied to the background subtraction result.

  // Histogram bins (Hue, Saturation)
  int BinsH;
  int BinsS;

  // Object detection parameters
  float Fbg;    // Degree to remove background model (histogram). Larger remove more.
  float Fgain;  // Learning rate.

  int NumModel;         // Number of object models to be maintained.
  int NumFramesInHist;  // Number of frames to make an object histogram.

  // Object tracking parameters
  int NThreshold2;  // Threshold applied to back projection.
  int NErode2;
  int NDilate2;

  int NCalibBGFrames;  // Number of frames to make a background histogram.

  // Simplifying detected object and movement for easy use
  int ObjSW, ObjSH;  // Size of shrunken object data
  int MvSW, MvSH;  // Size of shrunken movement data

  TObjDetTrackBSPParams();
};
void WriteToYAML(const std::vector<TObjDetTrackBSPParams> &params, const std::string &file_name);
void ReadFromYAML(std::vector<TObjDetTrackBSPParams> &params, const std::string &file_name);
//-------------------------------------------------------------------------------------------

/* Object detection and tracking method based on
  background subtraction to detect object movement,
  histogram of hsv as an object model, and
  back projection for detecting object.
  BSP = Background Subtraction and Back Projection.
*/
class TObjDetTrackBSP
{
public:
  TObjDetTrackBSP()
    : mode_detect_(true)  {}

  // User defined identifier.
  std::string Name;

  void Init();
  void Step(const cv::Mat &frame);
  void Draw(cv::Mat &frame);
  void CalibBG(const std::vector<cv::Mat> &frames);

  // Add area around a point p to the object models.
  // The background model is NOT subtracted.
  void AddToModel(const cv::Mat &frame, const cv::Point &p);

  void StartDetect()  {mode_detect_= true;}
  void StopDetect()   {mode_detect_= false;}

  void ClearObject()  {hist_obj_.clear();}

  TObjDetTrackBSPParams& Params()  {return params_;}
  const TObjDetTrackBSPParams& Params() const {return params_;}

  // Raw object mask.
  const cv::Mat& ObjRaw() const {return mask_obj1_;}
  // Expanded object mask.
  const cv::Mat& ObjExp() const {return mask_obj2_;}
  // Movement points.
  const cv::Mat& Mv() const {return mask_mv_;}

  // Moment of object (ObjectRaw).
  //http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
  //https://en.wikipedia.org/wiki/Image_moment
  const cv::Moments& ObjMoments() const {return obj_mu_;}
  // Shrunken object.
  const cv::Mat& ObjS() const {return obj_;}
  // Shrunken movement.
  const cv::Mat& MvS() const {return mv_;}

private:
  TObjDetTrackBSPParams params_;
  cv::Ptr<cv::BackgroundSubtractorMOG2> bkg_sbtr_;

  cv::Mat hist_bg_;  // Background model (histogram).
  int i_frame_;  // Frame index.

  // Object model (histogram).
  // We maintain multiple object models as hist_obj_.
  // For every NumFramesInHist/NumModel frame, a new model is added to the front.
  // hist_obj_.back() is the main model.
  std::list<cv::Mat> hist_obj_;

  cv::Mat mask_bs_, mask_bser_;
  cv::Mat mask_obj1_, mask_obj2_, mask_mv_;

  // Moment of object (mask_obj1_)
  cv::Moments obj_mu_;
  // Shrunken object and movement data
  cv::Mat obj_, mv_;

  bool mode_detect_;
};
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
// Implementation
//-------------------------------------------------------------------------------------------

// Shrink image by area-based interpolation.
// Similar to cv::resize with INTER_AREA, but this function returns data as a float matrix.
static void Shrink(const cv::Mat &src, cv::Mat &dst, const cv::Size &size)
{
  dst.create(size, CV_32FC1);
  int dx(float(src.cols)/float(dst.cols));
  int dy(float(src.rows)/float(dst.rows));
  float area(dx*dy);
  for(int r(0),rend(dst.rows);r<rend;++r)
    for(int c(0),cend(dst.cols);c<cend;++c)
      dst.at<float>(r,c)= cv::sum(src(cv::Rect(c*dx,r*dy,dx,dy)))[0]/area;
}
//-------------------------------------------------------------------------------------------


TObjDetTrackBSPParams::TObjDetTrackBSPParams()
{
  // Resize image for speed up
  Width= 160;
  Height= 120;

  // Remove black background
  ThreshBlkH= 180;
  ThreshBlkS= 255;
  ThreshBlkV= 40;

  // Parameters of BackgroundSubtractorMOG2
  BS_History= 30.0;
  BS_VarThreshold= 10.0;
  BS_DetectShadows= true;

  // Background subtraction and post process
  NErode1= 0;  // Size of erode applied to the background subtraction result.

  // Histogram bins (Hue, Saturation)
  BinsH= 100 ;
  BinsS= 10  ;

  // Object detection parameters
  Fbg= 1.5  ;  // Degree to remove background model (histogram). Larger remove more.
  Fgain= 0.5;  // Learning rate.

  NumModel= 5         ;  // Number of object models to be maintained.
  NumFramesInHist= 200;  // Number of frames to make an object histogram.

  // Object tracking parameters
  NThreshold2= 150 ; // Threshold applied to back projection.
  NErode2= 2       ;
  NDilate2= 7      ;
  NCalibBGFrames= 3;

  // Simplifying detected object and movement for easy use
  ObjSW= 3;  ObjSH= 3;  // Size of shrunken object data
  MvSW= 3;  MvSH= 3;  // Size of shrunken movement data
}
//-------------------------------------------------------------------------------------------

void WriteToYAML(const std::vector<TObjDetTrackBSPParams> &params, const std::string &file_name)
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  fs<<"ObjDetTrack"<<"[";
  for(std::vector<TObjDetTrackBSPParams>::const_iterator itr(params.begin()),itr_end(params.end()); itr!=itr_end; ++itr)
  {
    fs<<"{";
    #define PROC_VAR(x)  fs<<#x<<itr->x;
  // Resize image for speed up
    PROC_VAR(Width              );
    PROC_VAR(Height             );
    PROC_VAR(ThreshBlkH         );
    PROC_VAR(ThreshBlkS         );
    PROC_VAR(ThreshBlkV         );
    PROC_VAR(BS_History         );
    PROC_VAR(BS_VarThreshold    );
    PROC_VAR(BS_DetectShadows   );
    PROC_VAR(NErode1            );
    PROC_VAR(BinsH              );
    PROC_VAR(BinsS              );
    PROC_VAR(Fbg                );
    PROC_VAR(Fgain              );
    PROC_VAR(NumModel           );
    PROC_VAR(NumFramesInHist    );
    PROC_VAR(NThreshold2        );
    PROC_VAR(NErode2            );
    PROC_VAR(NDilate2           );
    PROC_VAR(NCalibBGFrames     );
    PROC_VAR(ObjSW              );
    PROC_VAR(ObjSH              );
    PROC_VAR(MvSW               );
    PROC_VAR(MvSH               );
    fs<<"}";
    #undef PROC_VAR
  }
  fs<<"]";
  fs.release();
}
//-------------------------------------------------------------------------------------------

void ReadFromYAML(std::vector<TObjDetTrackBSPParams> &params, const std::string &file_name)
{
  params.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data= fs["ObjDetTrack"];
  for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
  {
    TObjDetTrackBSPParams cf;
    #define PROC_VAR(x)  if(!(*itr)[#x].empty())  (*itr)[#x]>>cf.x;
    PROC_VAR(Width              );
    PROC_VAR(Height             );
    PROC_VAR(ThreshBlkH         );
    PROC_VAR(ThreshBlkS         );
    PROC_VAR(ThreshBlkV         );
    PROC_VAR(BS_History         );
    PROC_VAR(BS_VarThreshold    );
    PROC_VAR(BS_DetectShadows   );
    PROC_VAR(NErode1            );
    PROC_VAR(BinsH              );
    PROC_VAR(BinsS              );
    PROC_VAR(Fbg                );
    PROC_VAR(Fgain              );
    PROC_VAR(NumModel           );
    PROC_VAR(NumFramesInHist    );
    PROC_VAR(NThreshold2        );
    PROC_VAR(NErode2            );
    PROC_VAR(NDilate2           );
    PROC_VAR(NCalibBGFrames     );
    PROC_VAR(ObjSW              );
    PROC_VAR(ObjSH              );
    PROC_VAR(MvSW               );
    PROC_VAR(MvSH               );
    #undef PROC_VAR
    params.push_back(cf);
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// class TObjDetTrackBSP
//-------------------------------------------------------------------------------------------

void TObjDetTrackBSP::Init()
{
  bkg_sbtr_= new cv::BackgroundSubtractorMOG2(params_.BS_History, params_.BS_VarThreshold, params_.BS_DetectShadows);
  i_frame_= 0;
}
//-------------------------------------------------------------------------------------------

void TObjDetTrackBSP::Step(const cv::Mat &frame)
{
  float h_range[]= {0, 180};
  float s_range[]= {0, 255};
  const float* ranges[]= {h_range, s_range};
  int channels[]= {0, 1}, n_channels= 2;

  cv::Mat frame_s;
  if(params_.Width*params_.Height>0)
    cv::resize(frame, frame_s, cv::Size(params_.Width,params_.Height), 0, 0, cv::INTER_NEAREST);
  else
    frame_s= frame;

  cv::Mat frame_hsv, mask_nonblack;
  cv::cvtColor(frame_s, frame_hsv, cv::COLOR_BGR2HSV);
  cv::inRange(frame_hsv, cv::Scalar(0, 0, 0), cv::Scalar(params_.ThreshBlkH, params_.ThreshBlkS, params_.ThreshBlkV), mask_nonblack);
  mask_nonblack= 255-mask_nonblack;

  (*bkg_sbtr_)(frame_s, mask_bs_, 1./float(params_.BS_History));
  mask_bs_&= mask_nonblack;  // Remove black background
  cv::erode(mask_bs_, mask_bser_, cv::Mat(), cv::Point(-1,-1), params_.NErode1);
  mask_bser_&= mask_nonblack;  // Remove black background

  // Object detection mode:
  if(mode_detect_ && !hist_bg_.empty())
  {
    // Get the Histogram and normalize it
    cv::Mat hist_tmp;
    int hist_size[]= {params_.BinsH, params_.BinsS};
    cv::calcHist(&frame_hsv, 1, channels, mask_bser_, hist_tmp, n_channels, hist_size, ranges, true, false);

    // Add/delete model
    int interval= params_.NumFramesInHist / params_.NumModel;
    if(hist_obj_.size()==0 || i_frame_%interval==0)
    {
      cv::Mat hist(hist_tmp.size(), hist_tmp.type());
      hist.setTo(0);
      hist_obj_.push_front(hist);
      if(hist_obj_.size() > params_.NumModel)  hist_obj_.pop_back();
    }
    // Update model
    for(std::list<cv::Mat>::iterator ho_itr(hist_obj_.begin()),ho_itr_end(hist_obj_.end());
        ho_itr!=ho_itr_end; ++ho_itr)
    {
      cv::Mat &hist(*ho_itr);
      // Add current histogram to the model, and
      // Normalize the histogram
      for(int h(0);h<params_.BinsH;++h) for(int s(0);s<params_.BinsS;++s)
      {
        hist.at<float>(h,s)+= params_.Fgain*std::max(0.0f,hist_tmp.at<float>(h,s)-params_.Fbg*hist_bg_.at<float>(h,s));
        if(hist.at<float>(h,s)>255)  hist.at<float>(h,s)= 255;
      }
    }
  }

  // Track object:
  if(hist_obj_.size()>0)
  {
    cv::Mat &hist(hist_obj_.back());
    cv::Mat backproj;
    cv::calcBackProject(&frame_hsv, 1, channels, hist, backproj, ranges, 1, true);
    backproj.copyTo(mask_obj1_);
    mask_obj1_&= mask_nonblack;  // Remove black background
    // std::cerr<<mask_obj1_<<std::endl;
    cv::threshold(mask_obj1_, mask_obj1_, params_.NThreshold2, 0, cv::THRESH_TOZERO);
    cv::erode(mask_obj1_,mask_obj1_,cv::Mat(),cv::Point(-1,-1), params_.NErode2);
    cv::dilate(mask_obj1_,mask_obj2_,cv::Mat(),cv::Point(-1,-1), params_.NDilate2);
    mask_mv_.setTo(0);

    mask_bs_.copyTo(mask_mv_, mask_obj2_);

    obj_mu_= cv::moments(mask_obj1_, /*binaryImage=*/true);

    // Shrink object and movement data
    // cv::resize(mask_obj1_, obj_, cv::Size(3,3), 0, 0, cv::INTER_AREA);
    // cv::resize(mask_mv_, mv_, cv::Size(3,3), 0, 0, cv::INTER_AREA);
    Shrink(mask_obj1_, obj_, cv::Size(params_.ObjSW, params_.ObjSH));
    Shrink(mask_mv_, mv_, cv::Size(params_.MvSW, params_.MvSH));
  }
  else
  {
    mask_mv_= cv::Mat();
    mask_obj1_= cv::Mat();
    mask_obj2_= cv::Mat();
  }

  ++i_frame_;
}
//-------------------------------------------------------------------------------------------

void TObjDetTrackBSP::Draw(cv::Mat &frame)
{
  // cv::Mat masks1[3]= {mask_bser_, 0.0*mask_bs_, mask_bs_}, cmask1;
  // cv::merge(masks1,3,cmask1);
  // frame+= cmask1;

  if(!mask_obj1_.empty() && !mask_bs_.empty() && !mask_mv_.empty())
  {
    cv::Mat mask_obj1_l, mask_obj2_l, mask_bs_l, mask_mv_l/*, mask_bser_l*/;
    if(params_.Width*params_.Height>0)
    {
      cv::resize(mask_obj1_, mask_obj1_l, frame.size(), 0, 0, cv::INTER_NEAREST);
      cv::resize(mask_obj2_, mask_obj2_l, frame.size(), 0, 0, cv::INTER_NEAREST);
      cv::resize(mask_bs_, mask_bs_l, frame.size(), 0, 0, cv::INTER_NEAREST);
      cv::resize(mask_mv_, mask_mv_l, frame.size(), 0, 0, cv::INTER_NEAREST);
      // cv::resize(mask_bser_, mask_bser_l, frame.size(), 0, 0, cv::INTER_NEAREST);
    }
    else
    {
      mask_obj1_l= mask_obj1_;
      mask_obj2_l= mask_obj2_;
      mask_bs_l= mask_bs_;
      mask_mv_l= mask_mv_;
      // mask_bser_l= mask_bser_;
    }

    cv::Mat masks2[3]= {0.5*mask_obj1_l+0.5*mask_obj2_l, 0.2*mask_bs_l, 0.5*mask_mv_l}, cmask2;
    cv::merge(masks2,3,cmask2);
    frame+= cmask2;

    // Visualize momenr (obj_mu_):
    // const cv::Moments &mu(obj_mu_);
    // if(mu.m00>0.0)
    // {
      // cv::Point2f center(mu.m10/mu.m00, mu.m01/mu.m00);
      // float angle= 0.5 * std::atan2(2.0*mu.mu11/mu.m00, (mu.mu20/mu.m00 - mu.mu02/mu.m00));
      // std::cerr<<mu.m00<<"  "<<angle*180.0/M_PI<<"  "<<center<<std::endl;
      // cv::line(frame, center, center+100.0*cv::Point2f(std::cos(angle),std::sin(angle)), cv::Scalar(0,0,255,128),10,8,0);
    // }

    // Visualize shrunken object and movement data (obj_, mv_):
    // std::cerr<<"obj="<<obj_.reshape(1,params_.ObjSW*params_.ObjSH)<<std::endl;
    // std::cerr<<"mv="<<mv_.reshape(1,params_.MvSW*params_.MvSH)<<std::endl;
    cv::Mat obj2, mv2;
    cv::resize(obj_, obj2, frame.size(), 0, 0, cv::INTER_NEAREST);
    cv::resize(mv_, mv2, frame.size(), 0, 0, cv::INTER_NEAREST);
    obj2.convertTo(obj2, CV_8UC1);
    mv2.convertTo(mv2, CV_8UC1);
    cv::Mat obj2s[3]= {0.5*obj2, 0.0*mv2, 0.5*mv2}, obj2c;
    cv::merge(obj2s,3,obj2c);
    frame+= obj2c;
  }
}
//-------------------------------------------------------------------------------------------

void TObjDetTrackBSP::CalibBG(const std::vector<cv::Mat> &frames)
{
  hist_bg_= cv::Mat();
  ClearObject();
  if(frames.size()==0)  return;

  std::cerr<<"Calibrating BG..."<<std::endl;
  float h_range[]= {0, 180};
  float s_range[]= {0, 255};
  const float* ranges[]= {h_range, s_range};
  int channels[]= {0, 1}, n_channels= 2;
  int hist_size[]= {params_.BinsH, params_.BinsS};

  cv::Mat frame_s, frame_hsv;
  cv::Mat hist_tmp;
  for(int i(0),i_end(frames.size()); i<i_end; ++i)
  {
    if(params_.Width*params_.Height>0)
      cv::resize(frames[i], frame_s, cv::Size(params_.Width,params_.Height));
    else
      frame_s= frames[i];
    cv::cvtColor(frame_s, frame_hsv, cv::COLOR_BGR2HSV);
    cv::calcHist(&frame_hsv, 1, channels, cv::Mat(), hist_tmp, n_channels, hist_size, ranges, true, false);
    if(i==0)
    {
      hist_tmp.copyTo(hist_bg_);
    }
    else
    {
      for(int h(0);h<params_.BinsH;++h) for(int s(0);s<params_.BinsS;++s)
        hist_bg_.at<float>(h,s)+= hist_tmp.at<float>(h,s);
    }
  }
  float f= 1.0/float(frames.size());
  for(int h(0);h<params_.BinsH;++h) for(int s(0);s<params_.BinsS;++s)
    hist_bg_.at<float>(h,s)*= f;
}
//-------------------------------------------------------------------------------------------

// Add area around a point p to the object models.
// The background model is NOT subtracted.
void TObjDetTrackBSP::AddToModel(const cv::Mat &frame, const cv::Point &p)
{
  if(hist_obj_.size()==0)  return;

  std::cerr<<"Modifying object model..."<<std::endl;

  // Get mask of points around the point
  cv::Mat mask= cv::Mat::zeros(frame.rows+2, frame.cols+2, CV_8UC1);
  cv::Scalar new_val(120,120,120);
  int lo(20), up(20);
  int new_mask_val= 255;
  int connectivity= 8;
  int flags= connectivity + (new_mask_val << 8 ) + cv::FLOODFILL_FIXED_RANGE + cv::FLOODFILL_MASK_ONLY;
  cv::floodFill(frame, mask, p, new_val, 0, cv::Scalar(lo,lo,lo), cv::Scalar(up,up,up), flags);
  mask= mask(cv::Range(1, mask.rows-1), cv::Range(1, mask.cols-1));

  // Get histogram
  float h_range[]= {0, 180};
  float s_range[]= {0, 255};
  const float* ranges[]= {h_range, s_range};
  int channels[]= {0, 1}, n_channels= 2;
  int hist_size[]= {params_.BinsH, params_.BinsS};
  cv::Mat frame_s, frame_hsv, mask_s;
  cv::Mat hist_tmp;
  if(params_.Width*params_.Height>0)
  {
    cv::resize(frame, frame_s, cv::Size(params_.Width,params_.Height));
    cv::resize(mask, mask_s, cv::Size(params_.Width,params_.Height));
  }
  else
  {
    frame_s= frame;
    mask_s= mask;
  }
  cv::cvtColor(frame_s, frame_hsv, cv::COLOR_BGR2HSV);
  cv::calcHist(&frame_hsv, 1, channels, mask_s, hist_tmp, n_channels, hist_size, ranges, true, false);

  // Update model
  for(std::list<cv::Mat>::iterator ho_itr(hist_obj_.begin()),ho_itr_end(hist_obj_.end());
      ho_itr!=ho_itr_end; ++ho_itr)
  {
    cv::Mat &hist(*ho_itr);
    // Add current histogram to the model, and
    // Normalize the histogram
    for(int h(0);h<params_.BinsH;++h) for(int s(0);s<params_.BinsS;++s)
    {
      hist.at<float>(h,s)+= hist_tmp.at<float>(h,s);
      if(hist.at<float>(h,s)>255)  hist.at<float>(h,s)= 255;
    }
  }
}
//-------------------------------------------------------------------------------------------

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

int BS_History(1), Fbg20(1), Fgain20(1);
void ModifyParams(int i, void *ptracker)
{
  TObjDetTrackBSP &tracker(*reinterpret_cast<TObjDetTrackBSP*>(ptracker));
  tracker.Params().BS_History= BS_History;
  tracker.Params().Fbg= float(Fbg20)/20.0;
  tracker.Params().Fgain= float(Fgain20)/20.0;
}

struct TMouseEventData
{
  cv::Mat &frame;
  TObjDetTrackBSP &tracker;
  TMouseEventData(cv::Mat &f, TObjDetTrackBSP &t) : frame(f), tracker(t) {}
};
void OnMouse(int event, int x, int y, int flags, void *data)
{
  if(event == cv::EVENT_LBUTTONDOWN)
  {
    TMouseEventData &d(*reinterpret_cast<TMouseEventData*>(data));
    d.tracker.AddToModel(d.frame, cv::Point(x,y));
  }
}

int main(int argc, char**argv)
{
  std::string cam("0");
  int n_rotate90(0);
  if(argc>1)  cam= argv[1];
  if(argc>2)  n_rotate90= atoi(argv[2]);

  cv::VideoCapture cap;
  cap= CapOpen(cam, /*width=*/320, /*height=*/240);
  if(!cap.isOpened())  return -1;


  TObjDetTrackBSP tracker;
  tracker.Init();

  cv::Mat frame, frame_src;

  cv::namedWindow("camera",1);
  TMouseEventData mouse_data(frame_src,tracker);
  cv::setMouseCallback("camera", OnMouse, &mouse_data);
  bool calib_mode(false), detecting_mode(true);

  TEasyVideoOut vout;
  vout.SetfilePrefix("/tmp/objtr");

  for(int f(0);;++f)
  {
// std::cerr<<"---"<<std::endl;
// double t0(GetCurrentTime()),t1(0.0);
// t1=GetCurrentTime(); std::cerr<<"C0 "<<float((t1-t0)*1000.0)<<std::endl; t0=t1;
    cap >> frame; // get a new frame from camera
//*DEBUG*/cv::circle(frame, cv::Point(160,-160), 200, cv::Scalar(0,0,0), -1);
    // if(f%2!=0)  continue;  // Adjust FPS for speed up
// t1=GetCurrentTime(); std::cerr<<"C1 "<<float((t1-t0)*1000.0)<<std::endl; t0=t1;

    Rotate90N(frame,frame,n_rotate90);
    frame.copyTo(frame_src);

    if(f>0)
    {
      tracker.Step(frame);
      frame*= 0.3;
      tracker.Draw(frame);
    }
// t1=GetCurrentTime(); std::cerr<<"C2 "<<float((t1-t0)*1000.0)<<std::endl; t0=t1;

    vout.Step(frame);
    vout.VizRec(frame);
    cv::imshow("camera", frame);
    char c(cv::waitKey(3));
    if(c=='\x1b'||c=='q') break;
    else if(char(c)=='W')  vout.Switch();
    else if(c=='r')
    {
      tracker.ClearObject();
    }
    else if(c=='d')
    {
      detecting_mode= !detecting_mode;
      if(detecting_mode)  tracker.StartDetect();
      else                tracker.StopDetect();
    }
    else if(c=='b' || f==0)
    {
      std::vector<cv::Mat> frames;
      for(int i(0); i<tracker.Params().NCalibBGFrames; ++i)
      {
        cap >> frame; // get a new frame from camera
        Rotate90N(frame,frame,n_rotate90);
        frames.push_back(frame.clone());
      }
      tracker.CalibBG(frames);
    }
    else if(c=='C' || c=='c')
    {
      calib_mode= !calib_mode;
      if(calib_mode)
      {
        BS_History= tracker.Params().BS_History;
        Fbg20= tracker.Params().Fbg*20.0;
        Fgain20= tracker.Params().Fgain*20.0;
        cv::createTrackbar( "History:", "camera", &BS_History, 100, &ModifyParams, &tracker);
        cv::createTrackbar( "20*f_bg:", "camera", &Fbg20, 100, &ModifyParams, &tracker);
        cv::createTrackbar( "20*f_gain:", "camera", &Fgain20, 100, &ModifyParams, &tracker);
        cv::createTrackbar( "N-Erode(1):", "camera", &tracker.Params().NErode1, 10, NULL);
        cv::createTrackbar( "N-Erode(2):", "camera", &tracker.Params().NErode2, 20, NULL);
        cv::createTrackbar( "N-Dilate(2):", "camera", &tracker.Params().NDilate2, 20, NULL);
        cv::createTrackbar( "Threshold(2):", "camera", &tracker.Params().NThreshold2, 255, NULL);
      }
      else
      {
        // Remove trackbars from window.
        cv::destroyWindow("camera");
        cv::namedWindow("camera",1);
      }
    }
    // usleep(10000);
// t1=GetCurrentTime(); std::cerr<<"C3 "<<float((t1-t0)*1000.0)<<std::endl; t0=t1;
  }
  return 0;
}
//-------------------------------------------------------------------------------------------
