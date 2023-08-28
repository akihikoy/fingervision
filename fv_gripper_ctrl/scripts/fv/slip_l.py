#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'A signal function to get a total amount of slip of nearby object (left sensor).'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  slip= np.sum(fv_data.mv_s[LEFT])
  return slip
