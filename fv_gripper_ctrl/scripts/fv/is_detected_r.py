#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *

def Help():
  return '''A signal function to return if an object is detected (right sensor).
    Note that this sensor is used in center/orientation/da_area/d_center/da_orientation sensors to check if the values are effective.
    Thus the parameter min_obj_area is common for those sensors.'''

def SetDefaultParams(fvg):
  fvg.fv_ctrl_param.min_obj_area= 0.05  #Minimum object area to detect an object (common for both sensors).

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  is_detected= fv_data.obj_area_filtered[RIGHT]>fvg.fv_ctrl_param.min_obj_area
  return is_detected
