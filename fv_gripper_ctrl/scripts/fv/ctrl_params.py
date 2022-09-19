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
def Set(ct):
  if 'fv_ctrl' not in ct.cnt:
    ct.cnt.fv_ctrl= TContainer()

  #Common control parameters:
  ct.cnt.fv_ctrl.min_gstep= 0.0005
  ct.cnt.fv_ctrl.effort= 100.0

  #Parameters used in fv.grasp:
  ct.cnt.fv_ctrl.grasp_th= 3  #Threshold to stop.
  ct.cnt.fv_ctrl.grasp_filter_len= 4  #Temporal filter length.
  ct.cnt.fv_ctrl.grasp_dstate_th= 3  #Threshold of discrete state.

  #Parameters used in fv.hold:
  ct.cnt.fv_ctrl.hold_sensitivity_slip= 0.08  #Sensitivity of slip detection (smaller is more sensitive).
  ct.cnt.fv_ctrl.hold_sensitivity_oc= 0.2  #Sensitivity of object-center-movement detection (smaller is more sensitive).
  ct.cnt.fv_ctrl.hold_sensitivity_oo= 0.5  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
  ct.cnt.fv_ctrl.hold_sensitivity_oa= 0.4  #Sensitivity of object-area-change detection (smaller is more sensitive).

  #Parameters used in fv.openif:
  ct.cnt.fv_ctrl.openif_sensitivity_slip= 0.6  #Sensitivity of slip detection (smaller is more sensitive).
  ct.cnt.fv_ctrl.openif_sensitivity_oc= 0.4  #Sensitivity of object-center-movement detection (smaller is more sensitive).
  ct.cnt.fv_ctrl.openif_sensitivity_oo= 4.0  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
  ct.cnt.fv_ctrl.openif_sensitivity_oa= 0.6  #Sensitivity of object-area-change detection (smaller is more sensitive).
  ct.cnt.fv_ctrl.openif_sensitivity_force= 0.9  #Sensitivity of each force element; if the norm of force change is larger than this threshold, the point is counted as a force change point.
  ct.cnt.fv_ctrl.openif_nforce_threshold= 20  #Threshold of number of force changing points to open the gripper.
  ct.cnt.fv_ctrl.openif_dw_grip= 0.02  #Displacement of gripper movement.

  #Load fv_ctrl parameters from files.
  CONFIG_FILE= 'fv_ctrl.yaml'
  if 'config_path' not in ct.cnt:
    ct.cnt.config_path= [os.path.join(os.environ['HOME'],subdir) for subdir in ['config/','data/config/',]]
  ctrl_params= {}
  for dir_path in ct.cnt.config_path:
    file_path= os.path.join(dir_path,CONFIG_FILE)
    if os.path.exists(file_path):
      InsertDict(ctrl_params, LoadYAML(file_path))
  for k,v in ctrl_params.iteritems():
    ct.cnt.fv_ctrl[k]= v
