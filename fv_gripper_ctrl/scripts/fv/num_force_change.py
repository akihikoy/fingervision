#!/usr/bin/python
from ay_py.core import *

def Help():
  return 'A signal function to get a number of markers whose force changed from the initial.'

def SetDefaultParams(fvg):
  fvg.fv_ctrl_param.force_change_sensitivity= 0.9  #Sensitivity of each force element; if the norm of force change is larger than this threshold, the point is counted as a force change point.

def Reset(fvg):
  fvg.cnt.force_array0= copy.deepcopy(fvg.fv_data.force_array)

def Get(fvg, fv_data):
  force_diff_array
  num_change= sum([1 if Dist(f[:3],f0[:3])>fvg.fv_ctrl_param.force_change_sensitivity else 0
                   for side in (0,1)
                   for f,f0 in zip(fv_data.force_array[side],fvg.cnt.force_array0[side]) ])
  return num_change
