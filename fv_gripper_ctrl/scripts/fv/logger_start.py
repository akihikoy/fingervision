#!/usr/bin/python3
from ay_py.core import *
from ay_py.ros import *

def Help():
  return '''Start or resume logging.
    When starting from a paused state, resume logging to the same file.
    Otherwise, create a new log file and start logging.'''

def Run(fvg):
  fvg.logger.Start()
