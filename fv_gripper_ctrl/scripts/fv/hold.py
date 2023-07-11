#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
slip= SmartImportReload('fv.slip')
d_center_norm= SmartImportReload('fv.d_center_norm')
d_orientation= SmartImportReload('fv.d_orientation')
d_area= SmartImportReload('fv.d_area')

def SetDefaultParams(fvg):
  #Parameters used in fv.hold:
  fvg.fv_ctrl_param.hold_sensitivity_slip= 0.08  #Sensitivity of slip detection (smaller is more sensitive).
  fvg.fv_ctrl_param.hold_sensitivity_oc= 0.2  #Sensitivity of object-center-movement detection (smaller is more sensitive).
  fvg.fv_ctrl_param.hold_sensitivity_oo= 0.5  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
  fvg.fv_ctrl_param.hold_sensitivity_oa= 0.4  #Sensitivity of object-area-change detection (smaller is more sensitive).

'''
Slip-based holding.
Closing gripper if slip is detected.
'''
def Loop(fvg):
  fv_data= fvg.fv.data
  slip_detect2= lambda: ((slip.Get(fvg,fv_data)>fvg.fv_ctrl_param.hold_sensitivity_slip,
                          d_center_norm.Get(fvg,fv_data)>fvg.fv_ctrl_param.hold_sensitivity_oc,
                          d_orientation.Get(fvg,fv_data)>fvg.fv_ctrl_param.hold_sensitivity_oo,
                          d_area.Get(fvg,fv_data)>fvg.fv_ctrl_param.hold_sensitivity_oa))

  #Stop object detection
  fvg.fv.CallSrv('stop_detect_obj')

  #if fvg.gripper.Is('DxlGripper'):
    #fvg.gripper.StartHolding()

  g_pos= fvg.GripperPosition()
  while fvg.script_is_active and not rospy.is_shutdown():
    slips= slip_detect2()
    if any(slips):
      print 'slip',slips,rospy.Time.now().to_sec()
      g_pos-= fvg.fv_ctrl_param.min_gstep
      fvg.GripperMoveTo(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort, speed=1.0, blocking=False)
      for i in range(100):  #100
        if abs(fvg.GripperPosition()-g_pos)<0.5*fvg.fv_ctrl_param.min_gstep:  break
        rospy.sleep(0.0001)
      g_pos= fvg.GripperPosition()
    else:
      rospy.sleep(0.001)
    rospy.sleep(0.04)  #0.02

  #if fvg.gripper.Is('DxlGripper'):
    #fvg.gripper.StopHolding()
