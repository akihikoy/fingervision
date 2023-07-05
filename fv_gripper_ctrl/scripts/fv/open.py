#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
import tf
import sensor_msgs.msg

'''
Opening gripper with post process.
g_pos: Target gripper position. Default: Current position + 0.02.
'''
def Run(fvg, g_pos=None, blocking=False):
  if g_pos is None:  g_pos= fvg.GripperPosition()+0.02
  fvg.StopScript()
  fvg.GripperMoveTo(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort, blocking=blocking)
  fvg.fv.CallSrv('start_detect_obj')
