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
PYBIND11_MODULE(fv, m)
{
  m.doc() = "FingerVision library for Python (test).";

  py::class_<TBlobMove>(m, "TBlobMove")
    .def_readwrite("Pox", &TBlobMove::Pox)
    .def_readwrite("Poy", &TBlobMove::Poy)
    .def_readwrite("So" , &TBlobMove::So )
    .def_readwrite("DPx", &TBlobMove::DPx)
    .def_readwrite("DPy", &TBlobMove::DPy)
    .def_readwrite("DS" , &TBlobMove::DS )
    .def("__repr__", [](const TBlobMove& d){return d.ToString();});

  py::class_<TBlobMoves>(m, "TBlobMoves")
    .def_readwrite("time_stamp"  , &TBlobMoves::time_stamp  )
    .def_readwrite("frame_id"    , &TBlobMoves::frame_id    )
    .def_readwrite("camera_index", &TBlobMoves::camera_index)
    .def_readwrite("camera_name" , &TBlobMoves::camera_name )
    .def_readwrite("width"       , &TBlobMoves::width       )
    .def_readwrite("height"      , &TBlobMoves::height      )
    .def_readwrite("data"        , &TBlobMoves::data        )
    .def("__repr__", [](const TBlobMoves& d){return d.ToString();});

  py::class_<TProxVision>(m, "TProxVision")
    .def_readwrite("time_stamp"  , &TProxVision::time_stamp  )
    .def_readwrite("frame_id"    , &TProxVision::frame_id    )
    .def_readwrite("camera_index", &TProxVision::camera_index)
    .def_readwrite("camera_name" , &TProxVision::camera_name )
    .def_readwrite("width"       , &TProxVision::width       )
    .def_readwrite("height"      , &TProxVision::height      )
    .def_readwrite("ObjM_m"      , &TProxVision::ObjM_m      )
    .def_readwrite("ObjM_mu"     , &TProxVision::ObjM_mu     )
    .def_readwrite("ObjM_nu"     , &TProxVision::ObjM_nu     )
    .def_readwrite("ObjS"        , &TProxVision::ObjS        )
    .def_readwrite("MvS"         , &TProxVision::MvS         )
    .def("__repr__", [](const TProxVision& d){return d.ToString();});;

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
  m.def("SetCalibrationRequest", &SetCalibrationRequest, "Set the calibration flag with the specified name of a window.");
  m.def("ReqCalibrate", &ReqCalibrate, "ReqCalibrate.");
  m.def("ReqInitialize", &ReqInitialize, "ReqInitialize.");
  m.def("SetDimLevel", &SetDimLevel, "SetDimLevel.");
  m.def("SetTrackbarMode", &SetTrackbarMode, "SetTrackbarMode.");
  m.def("SaveParameters", &SaveParameters, "SaveParameters.");
  m.def("LoadParameters", &LoadParameters, "LoadParameters.");
  m.def("SaveCalibration", &SaveCalibration, "SaveCalibration.");
  m.def("LoadCalibration", &LoadCalibration, "LoadCalibration.");
  m.def("HandleWindowVisibilityRequest", &HandleWindowVisibilityRequest, "Handle the window visibility request and return if the windows are visible.");
  m.def("DisplayImages", &DisplayImages, "Handle the window visibility request, display images with imshow, and run the key event handler."
        "return: false if shutdown is requested.");
  m.def("DisplayImage", &DisplayImage, "Display an image (window) specified by name.");
  m.def("GetDisplayImageList", &GetDisplayImageList, "Return a list of image (window) names to be displayed.");
  m.def("GetNumCameras", &GetNumCameras, "Return the number of cameras.");
  m.def("GetBlobMoves", &GetBlobMoves, "Return the latest BlobMoves data.",
        py::arg("i_cam"));
  m.def("GetProxVision", &GetProxVision, "Return the latest ProxVision data.",
        py::arg("i_cam"));
  m.def("StopThreads", &StopThreads, "Stop the image capture and the computer vision threads.");
  m.def("StartThreads", &StartThreads, "Start the image capture and the computer vision threads.",
        py::arg("pkg_dir")=".",
        py::arg("config")="config/cam1.yaml",
        py::arg("config_out")="out1.yaml",
        py::arg("blob_calib_prefix")="blob_",
        py::arg("objdet_model_prefix")="objdet_",
        py::arg("frame_skip")=0,
        py::arg("target_fps")=0,
        py::arg("capture_fps")=0,
        py::arg("windows_hidden")=false,
        py::arg("camera_auto_reopen")=true,
        py::arg("initial_obj_detect")=true);

}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
