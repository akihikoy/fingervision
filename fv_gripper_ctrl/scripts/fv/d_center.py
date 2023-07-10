#!/usr/bin/python
from ay_py.core import *

def Help():
  return 'A signal function to get a temporal difference of center position (2d) of nearby object.'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  d_center= np.mean(fv_data.d_obj_center_filtered,axis=0)  #TODO:FIXME:Is "mean" correct for this?
  return d_center
