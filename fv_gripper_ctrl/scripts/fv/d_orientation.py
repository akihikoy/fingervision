#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *
is_detected_l= SmartImportReload('fv.is_detected_l')
is_detected_r= SmartImportReload('fv.is_detected_r')

def Help():
  return 'A signal function to get a temporal difference of orientation of nearby object.'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  d_orientation= np.mean([np.abs(fv_data.d_obj_orientation_filtered[RIGHT]) if is_detected_r.Get(fvg,fv_data) else 0.0,
                          np.abs(fv_data.d_obj_orientation_filtered[LEFT])  if is_detected_l.Get(fvg,fv_data) else 0.0])
  return d_orientation
