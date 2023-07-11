#!/usr/bin/python
from ay_py.core import *

def Help():
  return 'A signal function to get a total amount of slip of nearby object.'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  slip= np.sum(np.sum(s) for s in fv_data.mv_s)
  return slip
