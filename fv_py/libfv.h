//-------------------------------------------------------------------------------------------
/*! \file    libfv.h
    \brief   Standalone FingerVision module.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Aug.27, 2022
*/
//-------------------------------------------------------------------------------------------
#ifndef libfv_h
#define libfv_h
//-------------------------------------------------------------------------------------------
#include <list>
#include <vector>
#include <string>
#include <sstream>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

// Equivalent to fingervision_msgs/BlobMove
struct TBlobMove
{
  // Original position
  double Pox;
  double Poy;

  // Original size
  double So;

  // Displacement of position
  double DPx;
  double DPy;

  // Displacement of size
  double DS;

  std::string ToString(const std::string &indent="") const
    {
      std::ostringstream oss;
      #define PRINT(v)  oss<<indent<<#v":"<<v<<std::endl
      PRINT(Pox);
      PRINT(Poy);
      PRINT(So );
      PRINT(DPx);
      PRINT(DPy);
      PRINT(DS );
      #undef PRINT
      return oss.str();
    }
};

// Equivalent to fingervision_msgs/BlobMoves
struct TBlobMoves
{
  double time_stamp;
  std::string frame_id;

  int camera_index;
  std::string camera_name;
  int width;
  int height;
  std::vector<TBlobMove> data;

  std::string ToString(const std::string &indent="") const
    {
      std::ostringstream oss;
      #define PRINT(v)  oss<<indent<<#v":"<<v<<std::endl
      PRINT(time_stamp  );
      PRINT(frame_id    );
      PRINT(camera_index);
      PRINT(camera_name );
      PRINT(width       );
      PRINT(height      );
      #undef PRINT
      oss<<indent<<"data:"<<std::endl;
      for(std::vector<TBlobMove>::const_iterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
      {
        oss<<indent<<"  -"<<std::endl;
        oss<<itr->ToString(indent+"    ");
      }
      return oss.str();
    }
};

// Equivalent to fingervision_msgs/ProxVision
struct TProxVision
{
  double time_stamp;
  std::string frame_id;

  int camera_index;
  std::string camera_name;
  int width;
  int height;

  // Moment of object
  // cf. http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
  // cf. https://en.wikipedia.org/wiki/Image_moment
  // Moment of object - spatial moments [m00, m10, m01, m20, m11, m02, m30, m21, m12, m03]
  std::vector<double>  ObjM_m;
  // Moment of object - central moments [mu20, mu11, mu02, mu30, mu21, mu12, mu03]
  std::vector<double>  ObjM_mu;
  // Moment of object - central normalized moments [nu20, nu11, nu02, nu30, nu21, nu12, nu03]
  std::vector<double>  ObjM_nu;
  // Shrunken object matrix [o(x0,y0),o(x1,y0),...,o(xDX,y0); o(x0,y1),o(x1,y1),...,o(xDX,y1); o(x0,yDY),o(x1,yDY),...,o(xDX,yDY)]
  std::vector<double>  ObjS;
  // Shrunken movement matrix [m(x0,y0),m(x1,y0),...,m(xDX,y0); m(x0,y1),m(x1,y1),...,m(xDX,y1); m(x0,yDY),m(x1,yDY),...,m(xDX,yDY)]
  std::vector<double>  MvS;

  std::string ToString(const std::string &indent="") const
    {
      std::ostringstream oss;
      std::string delim;
      #define PRINT(v)  oss<<indent<<#v":"<<v<<std::endl
      #define PRINTVEC(v)        \
          oss<<indent<<#v": [";  \
          delim= "";             \
          for(std::vector<double>::const_iterator itr(v.begin()),itr_end(v.end());itr!=itr_end;++itr)  \
          {                      \
            oss<<delim<<*itr;    \
            delim= ", ";         \
          }                      \
          oss<<"]"<<std::endl
      PRINT(time_stamp  );
      PRINT(frame_id    );
      PRINT(camera_index);
      PRINT(camera_name );
      PRINT(width       );
      PRINT(height      );
      PRINTVEC(ObjM_m   );
      PRINTVEC(ObjM_mu  );
      PRINTVEC(ObjM_nu  );
      PRINTVEC(ObjS     );
      PRINTVEC(MvS      );
      #undef PRINT
      #undef PRINTVEC
      return oss.str();
    }
};

bool IsShutdown();
void SetShutdown();
bool HandleKeyEvent();
void Pause();
void Resume();
void ShowWindows();
void HideWindows();
void StartRecord();
void StopRecord();
void SetVideoPrefix(const std::string &file_prefix);
void SetFrameSkip(int frame_skip);
/*Take a snapshot.
Filename(camera) = [prefix]-CameraName-TimeStamp[ext].
ext: File extension (.png, .jpg, etc.).
return: files (Snapshot file names). */
std::list<std::string> TakeSnapshot(const std::string &prefix, const std::string &ext);
void StopDetectObj();
void StartDetectObj();
void ClearObj();
// Set the calibration flag with the specified name of a window.
void SetCalibrationRequest(const std::string &name);
bool ReqCalibrate(const std::string &kind, const int &idx);
bool ReqInitialize(const std::string &kind, const int &idx);
bool SetDimLevel(const std::string &kind, const int &dim_idx_in);
bool SetTrackbarMode(const std::string &kind, const int &mode);
void SaveParameters(const std::string &file_name_in);
void LoadParameters(const std::string &file_names_in);
bool SaveCalibration(const std::string &kind, const int &idx);
bool LoadCalibration(const std::string &kind, const int &idx);
// Handle the window visibility request and return if the windows are visible.
bool HandleWindowVisibilityRequest();
// Handle the window visibility request, display images with imshow, and run the key event handler.
// return: false if shutdown is requested.
bool DisplayImages();
// Display an image (window) specified by name.
void DisplayImage(const std::string &name);
// Return a list of image (window) names to be displayed.
std::list<std::string> GetDisplayImageList();
// Return the number of cameras.
int GetNumCameras();
// Return the latest BlobMoves data.
TBlobMoves GetBlobMoves(int i_cam);
// Return the latest ProxVision data.
TProxVision GetProxVision(int i_cam);
void StopThreads();
void StartThreads(
    const std::string &pkg_dir=".",
    const std::string &config="config/cam1.yaml",
    const std::string &config_out="out1.yaml",
    const std::string &blob_calib_prefix="blob_",
    const std::string &objdet_model_prefix="objdet_",
    int frame_skip=0, int target_fps=0, int capture_fps=0,
    bool windows_hidden=false, bool camera_auto_reopen=true, bool initial_obj_detect=true);

//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // libfv_h
//-------------------------------------------------------------------------------------------
