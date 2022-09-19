#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
import tf
import sensor_msgs.msg
import fv
import ctrl_params

'''
In-hand manipulation.
'''

def ManipLoop(th_info, ct):
  ctrl_params.Set(ct)
  fv_data= ct.cnt.fv

  side= 0 if fv_data.obj_area[0]>fv_data.obj_area[1] else 1
  get_theta= lambda: fv_data.obj_orientation[side]

  theta0= get_theta()
  target_angle= DegToRad(20.0)
  thread_cond= lambda: th_info.IsRunning() and not rospy.is_shutdown()
  while thread_cond():
    if abs(theta0-get_theta())>target_angle:
      print 'Done! target=', RadToDeg(target_angle)
      g_pos-= ct.cnt.fv_ctrl.min_gstep
      ct.gripper.Move(pos=g_pos, max_effort=ct.cnt.fv_ctrl.effort, speed=1.0, blocking=False)
      break

    #Open gripper until slip is detected
    g_pos= ct.gripper.Position()
    while thread_cond() and sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])<0.05:
      g_pos+= ct.cnt.fv_ctrl.min_gstep
      ct.gripper.Move(pos=g_pos, max_effort=ct.cnt.fv_ctrl.effort, speed=1.0, blocking=False)
      for i in range(100):
        if abs(ct.gripper.Position()-g_pos)<0.5*ct.cnt.fv_ctrl.min_gstep:  break
        rospy.sleep(0.0001)
      for i in range(100):
        if sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>=0.05:  break
        rospy.sleep(0.001)
      g_pos= ct.gripper.Position()

    #Close gripper to stop slip
    g_pos= ct.gripper.Position()
    while thread_cond() and sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>0.05:
      g_pos-= ct.cnt.fv_ctrl.min_gstep
      ct.gripper.Move(pos=g_pos, max_effort=ct.cnt.fv_ctrl.effort, speed=1.0, blocking=False)
      for i in range(100):
        if abs(ct.gripper.Position()-g_pos)<0.5*ct.cnt.fv_ctrl.min_gstep:  break
        rospy.sleep(0.0001)
      g_pos= ct.gripper.Position()

    print RadToDeg(theta0-get_theta()), RadToDeg(get_theta())

def On(ct):
  if 'vs_inhand' in ct.thread_manager.thread_list:
    print 'vs_inhand is already on'

  if not fv.IsActive(ct):
    raise Exception('fv is not configured. Use fv.Setup beforehand.')

  CPrint(1,'Turn on:','vs_inhand')
  ct.thread_manager.Add(name='vs_inhand', target=lambda th_info: ManipLoop(th_info,ct))

def Off(ct):
  CPrint(2,'Turn off:','vs_inhand')
  ct.thread_manager.Stop(name='vs_inhand')

