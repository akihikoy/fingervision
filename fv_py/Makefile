.PHONY: all fv.so

all: fv.so

fv.so:
	g++ -g -Wall -O3 -o fv.so fv.cpp libfv.cpp ../fv_core/blob_tracker2.cpp ../fv_core/prox_vision.cpp ../../ay_vision/ay_vision/src/vision_util.cpp -I../../ay_vision/ay_vision/include/ -I../../ay_cpp/include/ -I../fv_core -I/usr/include/eigen3 `pkg-config --cflags --libs opencv4` -lboost_thread -lboost_system -shared -fPIC -I/usr/include/python3 `python3 -m pybind11 --includes`
