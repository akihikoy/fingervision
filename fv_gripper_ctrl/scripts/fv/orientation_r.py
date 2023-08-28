#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'A signal function to get an orientation of nearby object (right sensor).'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  orientation= fv_data.obj_orientation[RIGHT]
  return orientation
