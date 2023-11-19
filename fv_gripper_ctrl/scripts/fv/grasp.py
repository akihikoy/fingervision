#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
num_force_change= SmartImportReload('fv.num_force_change')

def Help():
  return 'Force-based grasp control. Closing gripper until it detects a force change.'

def SetDefaultParams(fvg):
  #Parameters used in fv.grasp:
  fvg.fv_ctrl_param.grasp_nforce_threshold= 20  #Threshold of number of force changing points to stop closing the gripper.

def Loop(fvg):
  fv_data= fvg.fv.data

  fvg.fv.CallSrv('stop_detect_obj')
  num_force_change.Reset(fvg)

  g_pos= fvg.GripperPosition()
  while fvg.script_is_active and not rospy.is_shutdown():
    num_fc= num_force_change.Get(fvg,fv_data)
    if num_fc>fvg.fv_ctrl_param.grasp_nforce_threshold:
      CPrint(2,'Detected num_force_change=,',num_fc)
      break
    g_pos-= fvg.fv_ctrl_param.min_gstep
    fvg.GripperMoveTo(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort, speed=1.0, blocking=False)
    rospy.sleep(0.01)  #TODO:FIXME:This value should be configurable.

