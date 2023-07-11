#!/usr/bin/python
from ay_py.core import *
d_center= SmartImportReload('fv.d_center')

def Help():
  return 'A signal function to get a temporal difference of center position (2d) of nearby object.'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  d_center_norm= np.linalg.norm(d_center.Get(fvg,fv_data))
  return d_center_norm
