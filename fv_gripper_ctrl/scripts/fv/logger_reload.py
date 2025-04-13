#!/usr/bin/python3
from ay_py.core import *
from ay_py.ros import *

def Help():
  return '''Reload the signal list from the signal list file.
    Note that this action is needed when updating the signal list file via GUI
    or direct edit.'''

def Run(fvg):
  fvg.logger.ReloadSignalList()
