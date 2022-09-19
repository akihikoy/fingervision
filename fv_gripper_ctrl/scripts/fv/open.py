#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
import tf
import sensor_msgs.msg
import fv
import ctrl_params
import grasp
import hold
import openif
import inhand

'''
Opening gripper with post process.
g_pos: Target gripper position. Default: Current position + 0.02.
'''
def Run(ct, g_pos=None, blocking=False):
  CPrint(2,'open is called.')
  if g_pos is None:  g_pos= ct.gripper.Position()+0.02
  ctrl_params.Set(ct)
  grasp.Off(ct)
  hold.Off(ct)
  openif.Off(ct)
  inhand.Off(ct)
  ct.gripper.Move(pos=g_pos, max_effort=ct.cnt.fv_ctrl.effort, blocking=blocking)
  fv.CallSrv(ct, 'start_detect_obj')

