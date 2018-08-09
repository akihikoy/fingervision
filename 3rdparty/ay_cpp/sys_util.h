//-------------------------------------------------------------------------------------------
/*! \file    sys_util.h
    \brief   C++ system utility.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.28, 2018
*/
//-------------------------------------------------------------------------------------------
#ifndef sys_util_h
#define sys_util_h
//-------------------------------------------------------------------------------------------
#include <fstream>
#include <inttypes.h>  // int64_t
#include <sys/time.h>  // gettimeofday
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

inline double GetCurrentTime(void)
{
  struct timeval time;
  gettimeofday (&time, NULL);
  return static_cast<double>(time.tv_sec) + static_cast<double>(time.tv_usec)*1.0e-6;
  // return ros::Time::now().toSec();
}
//-------------------------------------------------------------------------------------------

inline int64_t GetCurrentTimeL(void)
{
  struct timeval time;
  gettimeofday (&time, NULL);
  return time.tv_sec*1e6l + time.tv_usec;
}
//-------------------------------------------------------------------------------------------

/*! \brief check the filename exists */
inline bool FileExists(const std::string &filename)
{
  bool res(false);
  std::ifstream ifs (filename.c_str());
  res = ifs.is_open();
  ifs.close();
  return res;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // sys_util_h
//-------------------------------------------------------------------------------------------
