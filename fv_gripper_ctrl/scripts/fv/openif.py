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
import inhand
import open
import openif

'''
Opening gripper if an external force is applied.
'''

def OpeningLoop(th_info, ct):
  ctrl_params.Set(ct)

  #Observation objects:
  fv_data= ct.cnt.fv

  fv.CallSrv(ct, 'stop_detect_obj')

  fa0= copy.deepcopy(fv_data.force_array)
  #FIXME: This should be a distance of (x,y) or (x,y,z) (z is estimated by x,y though...). Do not use torque.
  n_change= lambda side: sum([1 if Dist(f[:6],f0[:6])>ct.cnt.fv_ctrl.openif_sensitivity_force else 0 for f,f0 in zip(fv_data.force_array[side],fa0[side])])
  #dth= 5
  slip_detect2= lambda: ((sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>ct.cnt.fv_ctrl.openif_sensitivity_slip,
                          np.max(fv_data.d_obj_center_filtered)>ct.cnt.fv_ctrl.openif_sensitivity_oc,
                          np.max(fv_data.d_obj_orientation_filtered)>ct.cnt.fv_ctrl.openif_sensitivity_oo,
                          np.max(fv_data.d_obj_area_filtered)>ct.cnt.fv_ctrl.openif_sensitivity_oa))

  while th_info.IsRunning() and not rospy.is_shutdown():
    if n_change(0)+n_change(1)>ct.cnt.fv_ctrl.openif_nforce_threshold:
      print 'Force is applied,',n_change(0)+n_change(1)
      ct.gripper.Move(pos=ct.gripper.Position()+ct.cnt.fv_ctrl.openif_dw_grip, max_effort=ct.cnt.fv_ctrl.effort)
      break
    elif any(slip_detect2()):
      print 'Slip is detected',slip_detect2()
      ct.gripper.Move(pos=ct.gripper.Position()+ct.cnt.fv_ctrl.openif_dw_grip, max_effort=ct.cnt.fv_ctrl.effort)
      break
    else:
      rospy.sleep(0.005)

  fv.CallSrv(ct, 'start_detect_obj')

#Turn on an opening thread.
def On(ct):
  if 'vs_openif' in ct.thread_manager.thread_list:
    print 'vs_openif is already on'

  if not fv.IsActive(ct):
    raise Exception('fv is not configured. Use fv.Setup beforehand.')

  grasp.Off(ct)
  hold.Off(ct)

  CPrint(1,'Turn on:','vs_openif')
  ct.thread_manager.Add(name='vs_openif', target=lambda th_info: OpeningLoop(th_info,ct))

#Stop an opening thread.
def Off(ct):
  CPrint(2,'Turn off:','vs_openif')
  ct.thread_manager.Stop(name='vs_openif')

