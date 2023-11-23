#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
force_l= SmartImportReload('fv.force_l')
force_r= SmartImportReload('fv.force_r')

def Help():
  return 'Set zero the force_l and force_r values.'

def Loop(fvg):
  fv_data= fvg.fv.data
  force_l.SetZero(fvg, fv_data)
  force_r.SetZero(fvg, fv_data)
