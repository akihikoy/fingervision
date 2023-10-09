#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *
is_detected_l= SmartImportReload('fv.is_detected_l')

def Help():
  return 'A signal function to get a center position (2d) of nearby object (left sensor).'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  center= list(fv_data.obj_center[LEFT]) if is_detected_l.Get(fvg,fv_data) else [0,0]
  return center

