#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'A signal function to get a total amount of slip of nearby object.'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  if None in fv_data.mv_s:  return None
  slip= np.sum(np.sum(s) for s in fv_data.mv_s)
  return slip
