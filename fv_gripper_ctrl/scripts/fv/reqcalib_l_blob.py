#!/usr/bin/python3
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'Request calibration (l_blob)'

def Run(fvg):
  fvg.fv.CallSrvL('req_calibrate','BlobTracker',0)
