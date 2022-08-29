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
#include <chrono>
#include <thread>
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

class TRateAdjuster
{
public:
  typedef double Duration;
  typedef double Time;
  TRateAdjuster(const double &frequency)
      : expected_next_start_(GetCurrentTime()),
        actual_prev_start_(GetCurrentTime()),
        expected_cycle_time_(1.0 / frequency),
        actual_cycle_time_(0.0)
    {}
  void Sleep()
    {
      const Time &expected_curr_start(expected_next_start_);
      Time expected_next_end= expected_curr_start + expected_cycle_time_;
      Time actual_prev_end= GetCurrentTime();
      if(actual_prev_end < expected_curr_start)  expected_next_end= actual_prev_end+expected_cycle_time_;
      Duration sleep_time= expected_next_end - actual_prev_end;
      actual_cycle_time_= actual_prev_end-actual_prev_start_;
      actual_prev_start_= actual_prev_end;
      expected_next_start_= expected_next_end;
      if(sleep_time <= Duration(0.0))
      {
        if(actual_prev_end > expected_next_end+expected_cycle_time_)  expected_next_start_= actual_prev_end;
        return;
      }
      std::this_thread::sleep_for(std::chrono::nanoseconds(int(sleep_time*1.0e9)));
    }
  void Reset()  {expected_next_start_= GetCurrentTime();}
  Duration ActualCycleTime() const {return actual_cycle_time_;}
  Duration ExpectedCycleTime() const {return expected_cycle_time_;}

private:
  Time expected_next_start_, actual_prev_start_;
  Duration expected_cycle_time_, actual_cycle_time_;
};
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
