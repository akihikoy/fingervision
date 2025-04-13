#!/usr/bin/python3
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'Pause logging.  To restart logging, use fv.logger_start.'

def Run(fvg):
  fvg.logger.Pause()
