#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *
is_detected_r= SmartImportReload('fv.is_detected_r')

def Help():
  return 'A signal function to get an orientation of nearby object (right sensor).'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  if fv_data.obj_orientation[RIGHT] is None:  return None
  orientation= fv_data.obj_orientation[RIGHT] if is_detected_r.Get(fvg,fv_data) else 0.0
  return orientation
