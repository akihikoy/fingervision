//-------------------------------------------------------------------------------------------
/*! \file    fv.cpp
    \brief   Python wrapper of libfv using pybind
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Aug.27, 2022

g++ -g -Wall -O3 -o fv.so fv.cpp libfv.cpp ../fv_core/blob_tracker2.cpp ../fv_core/prox_vision.cpp ../3rdparty/ay_vision/vision_util.cpp -I../3rdparty -I../fv_core -I/usr/include/eigen3 -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_video -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lboost_thread -lboost_system -shared -fPIC -I/usr/include/python2.7 `python -m pybind11 --includes`

*/
//-------------------------------------------------------------------------------------------
#include "libfv.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

namespace py= pybind11;
PYBIND11_PLUGIN(fv)
{
  py::module m("fv", "FingerVision library for Python (test).");

  m.def("IsShutdown", &IsShutdown, "Check if the program is shutting down.");
  m.def("SetShutdown", &SetShutdown, "Set the shutdown flag.");
  m.def("HandleKeyEvent", &HandleKeyEvent, "Handle the OpenCV key event on the image windows.");
  m.def("Pause", &Pause, "Pause the image processing.");
  m.def("Resume", &Resume, "Resume the image processing.");
  m.def("ShowWindows", &ShowWindows, "Show windows.");
  m.def("HideWindows", &HideWindows, "Hide windows.");
  m.def("StartRecord", &StartRecord, "Start recording the videos.");
  m.def("StopRecord", &StopRecord, "Stop recording the videos.");
  m.def("SetVideoPrefix", &SetVideoPrefix, "Set the prefix of the videos.",
        py::arg("file_prefix"));
  m.def("SetFrameSkip", &SetFrameSkip, "Set FrameSkip.",
        py::arg("frame_skip"));
  m.def("TakeSnapshot", &TakeSnapshot, "Take a snapshot."
        "Filename(camera) = [prefix]-CameraName-TimeStamp[ext]."
        "ext: File extension (.png, .jpg, etc.)."
        "return: files (Snapshot file names).",
        py::arg("prefix"), py::arg("ext"));
  m.def("StopDetectObj", &StopDetectObj, "Stop detecting the objects on ObjDetTracker.");
  m.def("StartDetectObj", &StartDetectObj, "Start detecting the objects on ObjDetTracker.");
  m.def("ClearObj", &ClearObj, "Clear the object model of ObjDetTracker.");
  m.def("DisplayImages", &DisplayImages, "Display images with imshow and run the key event handler."
        "return: false if shutdown is requested.");
  m.def("DisplayImage", &DisplayImage, "Display an image (window) specified by name.");
  m.def("GetDisplayImageList", &GetDisplayImageList, "Return a list of image (window) names to be displayed.");
  m.def("StopThreads", &StopThreads, "Stop the image capture and the computer vision threads.");
  m.def("StartThreads", &StartThreads, "Start the image capture and the computer vision threads.",
        py::arg("pkg_dir")=".",
        py::arg("config")="config/cam1.yaml",
        py::arg("blob_calib_prefix")="blob_",
        py::arg("frame_skip")=0,
        py::arg("target_fps")=0,
        py::arg("capture_fps")=0,
        py::arg("windows_hidden")=false,
        py::arg("camera_auto_reopen")=true);
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
