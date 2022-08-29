//-------------------------------------------------------------------------------------------
/*! \file    fv_test.cpp
    \brief   Test code of libfv.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Aug.29, 2022

g++ -g -Wall -O2 -o fv_test.out fv_test.cpp libfv.cpp ../fv_core/blob_tracker2.cpp ../fv_core/prox_vision.cpp ../3rdparty/ay_vision/vision_util.cpp -I../3rdparty -I../fv_core -I/usr/include/eigen3 -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_video -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lboost_thread -lboost_system
*/
//-------------------------------------------------------------------------------------------
#include "libfv.h"
#include "ay_cpp/sys_util.h"
//-------------------------------------------------------------------------------------------
using namespace trick;
int main(int argc, char**argv)
{
  std::string config_file((argc>1)?argv[1]:"");
  StartThreads(/*pkg_dir=*/".",/*config=*/config_file);

  TRateAdjuster rate(30);
  while(!IsShutdown())
  {
    DisplayImages();
    rate.Sleep();
  }

  StopThreads();
}
//-------------------------------------------------------------------------------------------
