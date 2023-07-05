#!/usr/bin/python
from ay_py.core import *

def Help():
  return 'A signal function to get a temporal difference of area of nearby object.'

def Reset(fvg):
  pass

def Get(fvg):
  fv_data= fvg.fv.data
  d_area= np.mean(fv_data.d_obj_area_filtered)
  return d_area
