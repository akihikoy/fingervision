#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
import tf
import sensor_msgs.msg
import ctrl_params

'''
Slip-based holding.
Closing gripper if slip is detected.
'''
def Loop(fvg):
  ctrl_params.Set(fvg)
  fv_data= fvg.fv.data
  #slip_detect1= lambda: (sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>fvg.fv_ctrl_param.hold_sensitivity_slip)
  slip_detect2= lambda: ((sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>fvg.fv_ctrl_param.hold_sensitivity_slip,
                          np.max(fv_data.d_obj_center_filtered)>fvg.fv_ctrl_param.hold_sensitivity_oc,
                          np.max(fv_data.d_obj_orientation_filtered)>fvg.fv_ctrl_param.hold_sensitivity_oo,
                          np.max(fv_data.d_obj_area_filtered)>fvg.fv_ctrl_param.hold_sensitivity_oa))

  #Stop object detection
  fvg.fv.CallSrv('stop_detect_obj')

  #if fvg.gripper.Is('DxlGripper'):
    #fvg.gripper.StartHolding()

  g_pos= fvg.gripper.Position()
  while fvg.script_is_active and not rospy.is_shutdown():
    if any(slip_detect2()):
      print 'slip',slip_detect2(),rospy.Time.now().to_sec()
      g_pos-= fvg.fv_ctrl_param.min_gstep
      fvg.gripper.Move(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort, speed=1.0, blocking=False)
      for i in range(100):  #100
        if abs(fvg.gripper.Position()-g_pos)<0.5*fvg.fv_ctrl_param.min_gstep:  break
        rospy.sleep(0.0001)
      g_pos= fvg.gripper.Position()
    else:
      rospy.sleep(0.001)
    rospy.sleep(0.04)  #0.02

  #if fvg.gripper.Is('DxlGripper'):
    #fvg.gripper.StopHolding()
