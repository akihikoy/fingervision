#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
import tf
import sensor_msgs.msg

'''
Set parameters for controllers with FingerVision.
'''
def Set(fvg):
  #Common control parameters:
  fvg.fv_ctrl_param.min_gstep= 0.0005
  fvg.fv_ctrl_param.effort= 100.0

  #Parameters used in fv.grasp:
  fvg.fv_ctrl_param.grasp_th= 3  #Threshold to stop.
  fvg.fv_ctrl_param.grasp_filter_len= 4  #Temporal filter length.
  fvg.fv_ctrl_param.grasp_dstate_th= 3  #Threshold of discrete state.

  #Parameters used in fv.hold:
  fvg.fv_ctrl_param.hold_sensitivity_slip= 0.08  #Sensitivity of slip detection (smaller is more sensitive).
  fvg.fv_ctrl_param.hold_sensitivity_oc= 0.2  #Sensitivity of object-center-movement detection (smaller is more sensitive).
  fvg.fv_ctrl_param.hold_sensitivity_oo= 0.5  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
  fvg.fv_ctrl_param.hold_sensitivity_oa= 0.4  #Sensitivity of object-area-change detection (smaller is more sensitive).

  #Parameters used in fv.openif:
  fvg.fv_ctrl_param.openif_sensitivity_slip= 0.6  #Sensitivity of slip detection (smaller is more sensitive).
  fvg.fv_ctrl_param.openif_sensitivity_oc= 0.4  #Sensitivity of object-center-movement detection (smaller is more sensitive).
  fvg.fv_ctrl_param.openif_sensitivity_oo= 4.0  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
  fvg.fv_ctrl_param.openif_sensitivity_oa= 0.6  #Sensitivity of object-area-change detection (smaller is more sensitive).
  fvg.fv_ctrl_param.openif_sensitivity_force= 0.9  #Sensitivity of each force element; if the norm of force change is larger than this threshold, the point is counted as a force change point.
  fvg.fv_ctrl_param.openif_nforce_threshold= 20  #Threshold of number of force changing points to open the gripper.
  fvg.fv_ctrl_param.openif_dw_grip= 0.02  #Displacement of gripper movement.

  #Load fv_ctrl parameters from files.
  CONFIG_FILE= 'fv_ctrl.yaml'
  ctrl_params= {}
  for dir_path in fvg.config_path:
    file_path= os.path.join(dir_path,CONFIG_FILE)
    if os.path.exists(file_path):
      InsertDict(ctrl_params, LoadYAML(file_path))
  for k,v in ctrl_params.iteritems():
    fvg.fv_ctrl_param[k]= v
