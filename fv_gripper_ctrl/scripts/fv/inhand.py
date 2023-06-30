#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
import tf
import sensor_msgs.msg
import ctrl_params

'''
In-hand manipulation.
'''

def ManipLoop(th_info, fvg):
  ctrl_params.Set(fvg)
  fv_data= fvg.fv.data

  side= 0 if fv_data.obj_area[0]>fv_data.obj_area[1] else 1
  get_theta= lambda: fv_data.obj_orientation[side]

  theta0= get_theta()
  target_angle= DegToRad(20.0)
  thread_cond= lambda: th_info.IsRunning() and not rospy.is_shutdown()
  while thread_cond():
    if abs(theta0-get_theta())>target_angle:
      print 'Done! target=', RadToDeg(target_angle)
      g_pos-= fvg.fv_ctrl_param.min_gstep
      fvg.gripper.Move(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort, speed=1.0, blocking=False)
      break

    #Open gripper until slip is detected
    g_pos= fvg.gripper.Position()
    while thread_cond() and sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])<0.05:
      g_pos+= fvg.fv_ctrl_param.min_gstep
      fvg.gripper.Move(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort, speed=1.0, blocking=False)
      for i in range(100):
        if abs(fvg.gripper.Position()-g_pos)<0.5*fvg.fv_ctrl_param.min_gstep:  break
        rospy.sleep(0.0001)
      for i in range(100):
        if sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>=0.05:  break
        rospy.sleep(0.001)
      g_pos= fvg.gripper.Position()

    #Close gripper to stop slip
    g_pos= fvg.gripper.Position()
    while thread_cond() and sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>0.05:
      g_pos-= fvg.fv_ctrl_param.min_gstep
      fvg.gripper.Move(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort, speed=1.0, blocking=False)
      for i in range(100):
        if abs(fvg.gripper.Position()-g_pos)<0.5*fvg.fv_ctrl_param.min_gstep:  break
        rospy.sleep(0.0001)
      g_pos= fvg.gripper.Position()

    print RadToDeg(theta0-get_theta()), RadToDeg(get_theta())

def On(fvg):
  if 'vs_inhand' in fvg.thread_manager.thread_list:
    print 'vs_inhand is already on'

  if not fvg.fv.IsActive():
    raise Exception('fv is not configured. Use fv.Setup beforehand.')

  CPrint(1,'Turn on:','vs_inhand')
  fvg.thread_manager.Add(name='vs_inhand', target=lambda th_info: ManipLoop(th_info,fvg))

def Off(fvg):
  CPrint(2,'Turn off:','vs_inhand')
  fvg.thread_manager.Stop(name='vs_inhand')

