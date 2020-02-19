//-------------------------------------------------------------------------------------------
/*! \file    cpp_util.h
    \brief   C++ utility.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.28, 2018
*/
//-------------------------------------------------------------------------------------------
#ifndef cpp_util_h
#define cpp_util_h
//-------------------------------------------------------------------------------------------
#include <sstream>
#include <cstdlib>  // strtol, atol
#include <cctype>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

template <typename t_value>
inline t_value Sq(const t_value &val)
{
  return val*val;
}
//-------------------------------------------------------------------------------------------

inline std::string ToString(const std::string &prefix, int num, const std::string &posix="")
{
  std::stringstream ss;
  ss<<prefix<<num<<posix;
  return ss.str();
}
//-------------------------------------------------------------------------------------------

inline bool IsInt(const std::string &s)
{
  if(s.empty() || std::isspace(s[0]))  return false;
  char *p;
  strtol(s.c_str(), &p, 10);
  return (*p == 0);
}
//-------------------------------------------------------------------------------------------

inline int ToInt(const std::string &s)
{
  return atol(s.c_str());
}
//-------------------------------------------------------------------------------------------

/* Find an index idx of a sorted vector vec such that vec[idx]<=val<vec[idx+1].
  We can insert val by vec.insert(vec.begin()+idx+1, val) with keeping the sort.
  If vec.size()==0, idx=-1.
  If val<vec[0], idx=-1.
  If vec[vec.size()-1]<=val, idx=vec.size()-1. */
template<typename t_vector>
int FindIndex(const t_vector &vec, typename t_vector::const_reference val)
{
  for(int idx(0),end(vec.size()); idx<end; ++idx)
    if(val<vec[idx])  return idx-1;
  return vec.size()-1;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // cpp_util_h
//-------------------------------------------------------------------------------------------
