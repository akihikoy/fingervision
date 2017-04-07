//-------------------------------------------------------------------------------------------
/*! \file    usb_stereo.h
    \brief   USB stereo processing.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Apr.04, 2016
*/
//-------------------------------------------------------------------------------------------
#ifndef usb_stereo_h
#define usb_stereo_h
//-------------------------------------------------------------------------------------------
#include "lfd_vision/vision_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/calib3d/calib3d.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

struct TCameraParams
{
  cv::Mat D1, K1, D2, K2;  // distortion, intrinsics
  cv::Mat R1, R2, P1, P2, Q;  // output of stereoRectify
    // R1 : 3x3 rectification transform (rotation matrix)
    // R2 : 3x3 rectification transform (rotation matrix)
    // P1 : 3x4 projection matrix
    // P2 : 3x4 projection matrix
    // Q : 4x4 disparity-to-depth mapping matrix
  cv::Mat R, T;  // rotation, translation between cameras
};

struct TStereoParams
{
  // For cv::StereoBM and cv::StereoSGBM
  int minDisparity;
  int numberOfDisparities;
  // For cv::StereoBM
  int preset;
  // For cv::StereoSGBM
  int SADWindowSize;
  int preFilterCap;
  int uniquenessRatio;
  int P1;
  int P2;
  int speckleWindowSize;
  int speckleRange;
  int disp12MaxDiff;
  bool fullDP;
  // Extension
  enum TStereoMethod {smBM=1, smSGBM=2};
  int StereoMethod;  // 1: StereoBM, 2: StereoSGBM
  bool GrayScale;
  enum TLensType {ltBasic=1, ltFisheye=2};
  int LensType;
};


//===========================================================================================
// Basic stereo class.
class TStereo
//===========================================================================================
{
public:
  TStereo()  {}

  // Load camera parameters from a YAML file.
  bool LoadCameraParametersFromYAML(const std::string &file_name);
  // Load stereo parameters from a YAML file.
  bool LoadConfigurationsFromYAML(const std::string &file_name);

  // cv::Size(width,height)
  void SetImageSize(cv::Size size_in, cv::Size size_out)
    {img_size_in_= size_in; img_size_out_= size_out;}

  // img_size_in_,img_size_out_ must be set before using this.
  void SetRecommendedStereoParams();

  // Initialize stereo operation.
  void Init();

  // Process stereo operation from left and right images.
  void Proc(const cv::Mat &frame_l, const cv::Mat &frame_r);

  // Rectify images for stereo process (used in Proc).
  void Rectify(cv::Mat &frame1, cv::Mat &frame2, bool gray_scale=true);
  // Rectify left(1) image.
  void RectifyL(cv::Mat &frame1, bool gray_scale=false);
  // Rectify right(2) image.
  void RectifyR(cv::Mat &frame2, bool gray_scale=false);

  // Reproject disparity to 3D. Use after Proc.
  void ReprojectTo3D();

  // Get and set camera parameters.
  TCameraParams& CameraParams(void)  {return cp_;}
  // Get and set stereo parameters.
  TStereoParams& StereoParams(void)  {return sp_;}
  // Get disparity image.
  const cv::Mat& Disparity(void) const {return disparity_;}
  // Get 3D (XYZ) image.
  const cv::Mat& XYZ(void) const {return xyz_;}
  // Get RGB image corresponding with XYZ.
  const cv::Mat& RGB(void) const {return rgb_;}
  // Get processed images.
  const cv::Mat& FrameL(void) const {return frame1_;}
  const cv::Mat& FrameR(void) const {return frame2_;}

private:
  cv::Size img_size_in_, img_size_out_;  // cv::Size(width,height)
  TCameraParams cp_;
  TStereoParams sp_;
  cv::StereoBM stereo_bm_;
  cv::StereoSGBM stereo_sgbm_;

  // For undistort rectify map
  cv::Mat map11_, map12_, map21_, map22_;

  cv::Mat disparity_, disparity_f32_;
  cv::Mat xyz_, rgb_;
  cv::Mat frame1_, frame2_;

};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // usb_stereo_h
//-------------------------------------------------------------------------------------------
