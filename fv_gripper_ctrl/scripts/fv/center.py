#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'A signal function to get a center position (2d) of nearby object.'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  obj_center= [oc for oc in fv_data.obj_center if oc is not None]
  center= np.mean(fv_data.obj_center,axis=0)
  return center

