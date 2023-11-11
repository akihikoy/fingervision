#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'A signal function to get a total amount of slip of nearby object (right sensor).'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  if fv_data.mv_s[RIGHT] is None:  return None
  slip= np.sum(fv_data.mv_s[RIGHT])
  return slip
