#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *

def Help():
  return 'Controller of no action (assuming an external controller).'

def Loop(fvg):
  while fvg.script_is_active and not rospy.is_shutdown():
    rospy.sleep(0.05)
