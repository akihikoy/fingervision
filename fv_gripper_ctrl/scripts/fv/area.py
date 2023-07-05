#!/usr/bin/python
from ay_py.core import *

def Help():
  return 'A signal function to get an area of nearby object.'

def Reset(fvg):
  pass

def Get(fvg):
  fv_data= fvg.fv.data
  area= np.mean(fv_data.obj_area_filtered)*100.0  #TODO:FIXME:Put the scale in the parameter set.
  return area

