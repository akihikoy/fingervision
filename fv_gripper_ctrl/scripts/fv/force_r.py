#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'A signal function to get an average force (right sensor).'

def Reset(fvg):
  pass

def Get(fvg, fv_data):
  force= fv_data.force[RIGHT]
  return list(force) if force is not None else None

