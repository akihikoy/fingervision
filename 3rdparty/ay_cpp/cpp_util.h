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
#include <vector>
#include <string>
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

template<typename t_value>
t_value Mean(const std::vector<t_value> &vec)
{
  t_value sum(0.0);
  for(typename std::vector<t_value>::const_iterator itr(vec.begin()),itr_end(vec.end());
      itr!=itr_end; ++itr)
    sum+= *itr;
  return sum/(t_value)(vec.size());
}
//-------------------------------------------------------------------------------------------

// Return a median of vec.
// NOTE: vec is modified (sorted).
template<typename t_value>
inline t_value Median(std::vector<t_value> &vec)
{
  std::sort(vec.begin(),vec.end());
  if(vec.size()%2==1)  return vec[(vec.size()-1)/2];
  return 0.5*(vec[vec.size()/2-1]+vec[vec.size()/2]);
}
//-------------------------------------------------------------------------------------------

// Return a q-th percentile of vec.
// NOTE: vec is modified (sorted).
// NOTE: This method calculates a "lower" value of percentile.
template<typename t_value>
inline t_value Percentile(std::vector<t_value> &vec, const double &q)
{
  std::sort(vec.begin(),vec.end());
  int nth= std::max(0, std::min(int(vec.size()-1), int(q*vec.size()-1)));
  return vec[nth];
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

// Split a string str by  delimiter delim.
inline void SplitString(const std::string &str, std::vector<std::string> &result, char delim=',')
{
  std::stringstream ss(str);
  result.clear();
  while(ss.good())
  {
    std::string substr;
    std::getline(ss, substr, delim);
    result.push_back(substr);
  }
}
// Split a string str by  delimiter delim.
inline std::vector<std::string> SplitString(const std::string &str, char delim=',')
{
  std::vector<std::string> result;
  SplitString(str, result, delim);
  return result;
}
//-------------------------------------------------------------------------------------------

// Join two pathes: base_dir+file_name.
// If file_name starts with '/', file_name is returned as an absolute path.
// Otherwise, base_dir/file_name is returned.
inline std::string PathJoin(const std::string base_dir, const std::string &file_name)
{
  std::string delim;
  if(file_name.size()>0&&file_name.front()=='/')
    return file_name;
  if(base_dir.size()>0&&base_dir.back()=='/')
    delim= "";
  else
    delim= "/";
  return base_dir+delim+file_name;
}
//-------------------------------------------------------------------------------------------

// Make a vector of path: [base_dir+f for f in file_names].
inline std::vector<std::string> PathJoin(const std::string base_dir, const std::vector<std::string> &file_names)
{
  std::vector<std::string> result;
  for(std::vector<std::string>::const_iterator fitr(file_names.begin()),fitr_end(file_names.end());
      fitr!=fitr_end; ++fitr)
  {
    result.push_back(PathJoin(base_dir,*fitr));
  }
  return result;
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // cpp_util_h
//-------------------------------------------------------------------------------------------
