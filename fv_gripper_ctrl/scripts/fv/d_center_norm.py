#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *
is_detected_l= SmartImportReload('fv.is_detected_l')
is_detected_r= SmartImportReload('fv.is_detected_r')

def Help():
  return 'A signal function to get a norm of temporal difference of center position (2d) of nearby object.'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  #d_center_norm= np.mean(np.linalg.norm(fv_data.d_obj_center_filtered,axis=1))
  d_center_norm= np.mean([np.linalg.norm(fv_data.d_obj_center_filtered[RIGHT]) if is_detected_r.Get(fvg,fv_data) else 0.0,
                          np.linalg.norm(fv_data.d_obj_center_filtered[LEFT])  if is_detected_l.Get(fvg,fv_data) else 0.0])
  return d_center_norm
