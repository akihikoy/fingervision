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
#include <string>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

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
// Display images with imshow and run the key event handler.
// return: false if shutdown is requested.
bool DisplayImages();
// Display an image (window) specified by name.
void DisplayImage(const std::string &name);
// Return a list of image (window) names to be displayed.
std::list<std::string> GetDisplayImageList();
void StopThreads();
void StartThreads(
    const std::string &pkg_dir=".",
    const std::string &config="config/cam1.yaml",
    const std::string &blob_calib_prefix="blob_",
    int frame_skip=0, int target_fps=0, int capture_fps=0, bool windows_hidden=false, bool camera_auto_reopen=true);

//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // libfv_h
//-------------------------------------------------------------------------------------------
