#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *
is_detected_r= SmartImportReload('fv.is_detected_r')

def Help():
  return 'A signal function to get a temporal difference of center position (2d) of nearby object (right sensor).'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  d_center= list(fv_data.d_obj_center_filtered[RIGHT]) if is_detected_r.Get(fvg,fv_data) else [0,0]
  return d_center
