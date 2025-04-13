#!/usr/bin/python3
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'Finish logging.  The file descriptor is closed.'

def Run(fvg):
  fvg.logger.Finish()
