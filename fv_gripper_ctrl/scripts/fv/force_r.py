#!/usr/bin/python
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'A signal function to get an average force (right sensor).'

def Reset(fvg):
  fvg.cnt.force_r_base= None

def SetZero(fvg, fv_data):
  fvg.cnt.force_r_base= np.array(fv_data.force[RIGHT])

def Get(fvg, fv_data):
  force= fv_data.force[RIGHT]
  if fvg.cnt.force_r_base is not None:
    force= force-fvg.cnt.force_r_base
  return list(force) if force is not None else None

