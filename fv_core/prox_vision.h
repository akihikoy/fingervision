//-------------------------------------------------------------------------------------------
/*! \file    prox_vision.h
    \brief   Slip and deformation detection by detecting and tracking a proximity object.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.19, 2017

cf. testl/cv/obj_det_track3.cpp
*/
//-------------------------------------------------------------------------------------------
#ifndef prox_vision_h
#define prox_vision_h
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/function.hpp>
#include <iostream>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

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
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // prox_vision_h
//-------------------------------------------------------------------------------------------
