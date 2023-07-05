#!/usr/bin/python
from ay_py.core import *

def Help():
  return 'A signal function to get a temporal difference of orientation of nearby object.'

def Reset(fvg):
  pass

def Get(fvg):
  fv_data= fvg.fv.data
  d_orientation= np.mean(fv_data.d_obj_orientation)  #TODO:FIXME:Is "mean" correct for this?
  return d_orientation
