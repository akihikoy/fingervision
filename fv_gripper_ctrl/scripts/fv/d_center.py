#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'A signal function to get a temporal difference of center position (2d) of nearby object.'

def SetDefaultParams(fvg):
  fvg.fv_ctrl_param.max_d_center_norm= 1.0  #Upper limit of d_center_norm.

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  d_center= np.mean(fv_data.d_obj_center_filtered,axis=0)
  d_center_norm= np.linalg.norm(d_center)
  if d_center_norm>fvg.fv_ctrl_param.max_d_center_norm:
    d_center*= fvg.fv_ctrl_param.max_d_center_norm/d_center_norm
  return d_center
