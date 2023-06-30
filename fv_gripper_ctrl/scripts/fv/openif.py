#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
import ctrl_params

'''
Opening gripper if an external force is applied.
'''
def Loop(fvg):
  ctrl_params.Set(fvg)
  fv_data= fvg.fv.data

  fvg.fv.CallSrv('stop_detect_obj')

  fa0= copy.deepcopy(fv_data.force_array)
  #FIXME: This should be a distance of (x,y) or (x,y,z) (z is estimated by x,y though...). Do not use torque.
  n_change= lambda side: sum([1 if Dist(f[:6],f0[:6])>fvg.fv_ctrl_param.openif_sensitivity_force else 0 for f,f0 in zip(fv_data.force_array[side],fa0[side])])
  #dth= 5
  slip_detect2= lambda: ((sum(fv_data.mv_s[0])+sum(fv_data.mv_s[1])>fvg.fv_ctrl_param.openif_sensitivity_slip,
                          np.max(fv_data.d_obj_center_filtered)>fvg.fv_ctrl_param.openif_sensitivity_oc,
                          np.max(fv_data.d_obj_orientation_filtered)>fvg.fv_ctrl_param.openif_sensitivity_oo,
                          np.max(fv_data.d_obj_area_filtered)>fvg.fv_ctrl_param.openif_sensitivity_oa))

  g_pos= fvg.GripperPosition()
  while fvg.script_is_active and not rospy.is_shutdown():
    if n_change(0)+n_change(1)>fvg.fv_ctrl_param.openif_nforce_threshold:
      print 'Force is applied,',n_change(0)+n_change(1)
      g_pos= fvg.GripperPosition()+fvg.fv_ctrl_param.openif_dw_grip
      fvg.GripperMoveTo(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort)
      break
    elif any(slip_detect2()):
      print 'Slip is detected',slip_detect2()
      g_pos= fvg.GripperPosition()+fvg.fv_ctrl_param.openif_dw_grip
      fvg.GripperMoveTo(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort)
      break
    else:
      rospy.sleep(0.005)

  fvg.fv.CallSrv('start_detect_obj')
