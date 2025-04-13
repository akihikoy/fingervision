#!/usr/bin/python3
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'Save calibration (l_blob)'

def Run(fvg):
  fvg.fv.CallSrvL('save_calibration','BlobTracker',0)
