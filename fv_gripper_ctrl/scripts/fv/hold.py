#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
slip= SmartImportReload('fv.slip')
da_center_norm= SmartImportReload('fv.da_center_norm')
da_area= SmartImportReload('fv.da_area')

def Help():
  return 'Slip-based hold control. Closing gripper if slip is detected.'

def SetDefaultParams(fvg):
  #Parameters used in fv.hold:
  fvg.fv_ctrl_param.hold_sensitivity_slip= 0.08  #Sensitivity of slip detection (smaller is more sensitive).
  fvg.fv_ctrl_param.hold_sensitivity_oc= 0.2  #Sensitivity of object-center-movement detection (smaller is more sensitive).
  fvg.fv_ctrl_param.hold_sensitivity_oa= 0.4  #Sensitivity of object-area-change detection (smaller is more sensitive).

def Loop(fvg):
  fv_data= fvg.fv.data
  slip_detect2= lambda: ((slip.Get(fvg,fv_data)>fvg.fv_ctrl_param.hold_sensitivity_slip,
                          da_center_norm.Get(fvg,fv_data)>fvg.fv_ctrl_param.hold_sensitivity_oc,
                          da_area.Get(fvg,fv_data)>fvg.fv_ctrl_param.hold_sensitivity_oa))

  #Stop object detection
  fvg.fv.CallSrv('stop_detect_obj')

  g_pos= fvg.GripperPosition()
  while fvg.script_is_active and not rospy.is_shutdown():
    slips= slip_detect2()
    if any(slips):
      CPrint(2,'(slip,da_center_norm(oc),da_area(oa))',slips)
      g_pos-= fvg.fv_ctrl_param.min_gstep
      fvg.GripperMoveTo(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort, speed=1.0, blocking=False)
      for i in range(100):  #100
        if abs(fvg.GripperPosition()-g_pos)<0.5*fvg.fv_ctrl_param.min_gstep:  break
        rospy.sleep(0.0001)
      g_pos= fvg.GripperPosition()
    else:
      rospy.sleep(0.001)
    rospy.sleep(0.04)  #0.02
