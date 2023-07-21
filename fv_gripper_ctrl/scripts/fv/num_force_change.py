#!/usr/bin/python
from ay_py.core import *

def Help():
  return 'A signal function to get a number of markers whose force changed from the initial.'

def SetDefaultParams(fvg):
  fvg.fv_ctrl_param.force_change_sensitivity= 0.9  #Sensitivity of each force element; if the norm of force change is larger than this threshold, the point is counted as a force change point.
  fvg.fv_ctrl_param.force_init_len= 10  #Length of initial force array to average as a basis to compute the force array change.

def Reset(fvg):
  fvg.cnt.force_array0= [[],[]]

def Get(fvg, fv_data):
  for side in (0,1):
    force_array= fv_data.force_array[side]
    if force_array is None: continue
    if len(fvg.cnt.force_array0[side])<fvg.fv_ctrl_param.force_init_len:
      fvg.cnt.force_array0[side].append(force_array)
  if any(len(fvg.cnt.force_array0[0])==0, len(fvg.cnt.force_array0[1])==0,
         len(fv_data.force_array[0])==0, len(fv_data.force_array[0])==0):
    return None
  fa0= [np.mean(fvg.cnt.force_array0[0],axis=0), np.mean(fvg.cnt.force_array0[1],axis=0)]
  num_change= sum([1 if Dist(f[:3],f0[:3])>fvg.fv_ctrl_param.force_change_sensitivity else 0
                   for side in (0,1)
                   for f,f0 in zip(fv_data.force_array[side],fa0[side]) ])
  return num_change
