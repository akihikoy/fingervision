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
Slip-based holding.
Closing gripper if slip is detected.
'''

def HoldLoop(th_info, ct):
  ctrl_params.Set(ct)
  fv_data= ct.cnt.fv
  #slip_detect1= lambda: (sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>ct.cnt.fv_ctrl.hold_sensitivity_slip)
  slip_detect2= lambda: ((sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>ct.cnt.fv_ctrl.hold_sensitivity_slip,
                          np.max(fv_data.d_obj_center_filtered)>ct.cnt.fv_ctrl.hold_sensitivity_oc,
                          np.max(fv_data.d_obj_orientation_filtered)>ct.cnt.fv_ctrl.hold_sensitivity_oo,
                          np.max(fv_data.d_obj_area_filtered)>ct.cnt.fv_ctrl.hold_sensitivity_oa))

  #Stop object detection
  fv.CallSrv(ct, 'stop_detect_obj')

  #if ct.gripper.Is('DxlGripper'):
    #ct.gripper.StartHolding()

  g_pos= ct.gripper.Position()
  while th_info.IsRunning() and not rospy.is_shutdown():
    if any(slip_detect2()):
      print 'slip',slip_detect2(),rospy.Time.now().to_sec()
      g_pos-= ct.cnt.fv_ctrl.min_gstep
      ct.gripper.Move(pos=g_pos, max_effort=ct.cnt.fv_ctrl.effort, speed=1.0, blocking=False)
      for i in range(100):  #100
        if abs(ct.gripper.Position()-g_pos)<0.5*ct.cnt.fv_ctrl.min_gstep:  break
        rospy.sleep(0.0001)
      g_pos= ct.gripper.Position()
    else:
      rospy.sleep(0.001)
    rospy.sleep(0.04)  #0.02

  #if ct.gripper.Is('DxlGripper'):
    #ct.gripper.StopHolding()

#Turn on a holding thread.
def On(ct):
  if 'vs_hold' in ct.thread_manager.thread_list:
    print 'vs_hold is already on'

  if not fv.IsActive(ct):
    raise Exception('fv is not configured. Use fv.Setup beforehand.')

  CPrint(1,'Turn on:','vs_hold')
  ct.thread_manager.Add(name='vs_hold', target=lambda th_info: HoldLoop(th_info,ct))

#Stop a holding thread.
def Off(ct):
  CPrint(2,'Turn off:','vs_hold')
  ct.thread_manager.Stop(name='vs_hold')

