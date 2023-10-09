#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'A signal function to get a center position (2d) of nearby object (right sensor).'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  center= list(fv_data.obj_center[RIGHT])  #-0.55,0
  return center

