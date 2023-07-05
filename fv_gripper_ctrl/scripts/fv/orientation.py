#!/usr/bin/python
from ay_py.core import *

def Help():
  return 'A signal function to get an orientation of nearby object.'

def Reset(fvg):
  pass

def Get(fvg):
  fv_data= fvg.fv.data
  orientation= np.mean(fv_data.obj_orientation)  #TODO:FIXME:Is "mean" correct for this?
  return orientation
