//-------------------------------------------------------------------------------------------
/*! \file    sys_util.h
    \brief   C++ system utility.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.28, 2018
    \version 0.2
    \date    Sep.30, 2018
             Revised for Windows.
*/
//-------------------------------------------------------------------------------------------
#ifndef sys_util_h
#define sys_util_h
//-------------------------------------------------------------------------------------------
#include <fstream>
#ifdef __linux__
  #include <inttypes.h>  // int64_t
  #include <sys/time.h>  // gettimeofday
#elif _WIN32
  #include <stdint.h>
  #define NOMINMAX  // Disabling min and max macros in Windows.h
  #include <Windows.h>
  #undef GetCurrentTime  // Remove the macro defined in Windows.h
#else
  #error OS detection failed
#endif
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

inline double GetCurrentTime(void)
{
#ifdef __linux__
  struct timeval time;
  gettimeofday (&time, NULL);
  return static_cast<double>(time.tv_sec) + static_cast<double>(time.tv_usec)*1.0e-6;
#elif _WIN32
  ULONGLONG UnbiasedInterruptTime;
  QueryUnbiasedInterruptTime(&UnbiasedInterruptTime);
  return (double)UnbiasedInterruptTime/(double)10000000;
#else
  #error OS detection failed
#endif
}
//-------------------------------------------------------------------------------------------

inline int64_t GetCurrentTimeL(void)
{
#ifdef __linux__
  struct timeval time;
  gettimeofday (&time, NULL);
  return time.tv_sec*1e6l + time.tv_usec;
#elif _WIN32
  ULONGLONG UnbiasedInterruptTime;
  QueryUnbiasedInterruptTime(&UnbiasedInterruptTime);
  return UnbiasedInterruptTime/10;
#else
  #error OS detection failed
#endif
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
