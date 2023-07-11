#!/usr/bin/python
from ay_py.core import *

def Help():
  return 'A signal function to get an orientation of nearby object.'

def Reset(fvg):
  fvg.cnt.force_array0= copy.deepcopy(fv_data.force_array)

def Get(fvg, fv_data):
  force_diff_array
  num_change= lambda side: sum([1 if Dist(f[:3],f0[:3])>fvg.fv_ctrl_param.openif_sensitivity_force else 0 for f,f0 in zip(fv_data.force_array[side],fvg.cnt.force_array0[side])])
  return num_change
