#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
slip= SmartImportReload('fv.slip')
da_center_norm= SmartImportReload('fv.da_center_norm')
da_area= SmartImportReload('fv.da_area')
num_force_change= SmartImportReload('fv.num_force_change')

def Help():
  return 'Open the gripper if the sensor detects force change or slip.'

def SetDefaultParams(fvg):
  #Parameters used in fv.openif:
  fvg.fv_ctrl_param.openif_sensitivity_slip= 0.6  #Sensitivity of slip detection (smaller is more sensitive).
  fvg.fv_ctrl_param.openif_sensitivity_oc= 0.4  #Sensitivity of object-center-movement detection (smaller is more sensitive).
  fvg.fv_ctrl_param.openif_sensitivity_oa= 0.6  #Sensitivity of object-area-change detection (smaller is more sensitive).
  fvg.fv_ctrl_param.openif_nforce_threshold= 20  #Threshold of number of force changing points to open the gripper.
  fvg.fv_ctrl_param.openif_dw_grip= 0.02  #Displacement of gripper movement.

def Loop(fvg):
  fv_data= fvg.fv.data

  fvg.fv.CallSrv('stop_detect_obj')
  num_force_change.Reset(fvg)

  slip_detect2= lambda: ((slip.Get(fvg,fv_data)>fvg.fv_ctrl_param.openif_sensitivity_slip,
                          da_center_norm.Get(fvg,fv_data)>fvg.fv_ctrl_param.openif_sensitivity_oc,
                          da_area.Get(fvg,fv_data)>fvg.fv_ctrl_param.openif_sensitivity_oa))

  g_pos= fvg.GripperPosition()
  while fvg.script_is_active and not rospy.is_shutdown():
    slips,num_fc= slip_detect2(),num_force_change.Get(fvg,fv_data)
    if num_fc>fvg.fv_ctrl_param.openif_nforce_threshold:
      CPrint(2,'Detected num_force_change=,',num_fc)
      g_pos= fvg.GripperPosition()+fvg.fv_ctrl_param.openif_dw_grip
      fvg.GripperMoveTo(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort)
      break
    elif any(slips):
      CPrint(2,'Detected (slip,da_center_norm(oc),da_area(oa))',slips)
      g_pos= fvg.GripperPosition()+fvg.fv_ctrl_param.openif_dw_grip
      fvg.GripperMoveTo(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort)
      break
    else:
      rospy.sleep(0.005)

  fvg.fv.CallSrv('start_detect_obj')
