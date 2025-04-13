#!/usr/bin/python3
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'Request calibration (r_blob)'

def Run(fvg):
  fvg.fv.CallSrvR('req_calibrate','BlobTracker',0)
