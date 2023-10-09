#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'A signal function to get an area of nearby object (mean of left and right sensors).'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  area= np.mean(fv_data.obj_area_filtered)
  return area

