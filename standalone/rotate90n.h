//-------------------------------------------------------------------------------------------
/*! \file    rotate90n.h
    \brief   Rotate an image 90deg x n-times.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.09, 2016
*/
//-------------------------------------------------------------------------------------------
#ifndef rotate90n_h
#define rotate90n_h
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

/* Rotate 90, 180, 270 degrees
  ref. http://stackoverflow.com/questions/16265673/rotate-image-by-90-180-or-270-degrees
  transpose+flip v.s. warpAffine?
  http://stackoverflow.com/questions/15043152/rotate-opencv-matrix-by-90-180-270-degrees
  transpose+flip is 10x faster?
  https://gist.github.com/gcpantazis/6092838
*/
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
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // rotate90n_h
//-------------------------------------------------------------------------------------------
