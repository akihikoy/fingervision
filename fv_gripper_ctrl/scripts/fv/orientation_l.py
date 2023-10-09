#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *
is_detected_l= SmartImportReload('fv.is_detected_l')

def Help():
  return 'A signal function to get an orientation of nearby object (left sensor).'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  orientation= fv_data.obj_orientation[LEFT] if is_detected_l.Get(fvg,fv_data) else 0.0
  return orientation
