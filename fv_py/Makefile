.PHONY: all fv.so

all: fv.so

fv.so:
	g++ -g -Wall -O3 -o fv.so fv.cpp libfv.cpp ../fv_core/blob_tracker2.cpp ../fv_core/prox_vision.cpp ../3rdparty/ay_vision/vision_util.cpp -I../3rdparty -I../fv_core -I/usr/include/eigen3 -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_video -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lboost_thread -lboost_system -shared -fPIC -I/usr/include/python2.7 `python -m pybind11 --includes`
