#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
import tf
import sensor_msgs.msg

def SetDefaultParams(fvg):
  #Parameters used in fv.grasp:
  fvg.fv_ctrl_param.grasp_th= 3  #Threshold to stop.
  fvg.fv_ctrl_param.grasp_filter_len= 4  #Temporal filter length.
  fvg.fv_ctrl_param.grasp_dstate_th= 3  #Threshold of discrete state.

'''
Force-based grasping.
Closing gripper until it finds a force.
'''

#Force change detector class.
class TForceChangeDetector(object):
  #Setup.  fv_data: dictionary of observation that is expected to be updated by other thread.
  #th: threshold to stop, filter_len: temporal filter length.
  def __init__(self, fv_data, th=3, filter_len=4, dstate_th=3):
    self.fv_data= fv_data
    self.dth= th
    self.smoothing_filter_len= filter_len
    self.dstate_th= dstate_th

  #Get discrete state for robust force change detection.
  def GetDState(self,side):
    #return fv_data.dstate[side]
    return len([ds for ds in self.fv_data.dstate_array[side] if ds>=self.dstate_th])

  #Initialize the internal state.
  def Init(self):
    #Correct some data for temporal filtering.
    #self.tm_last= rospy.Time.now()
    self.dstate_list= [[],[]]  #list of self.GetDState(0) and self.GetDState(1)
    for side in (0,1):
      self.dstate_list[side].append(self.GetDState(side))
    self.tm_last= max(self.fv_data.tm_last_topic[:2])
    self.mean= Median
    self.is_initialized= False
    self.is_detected= False

  #Update dstate_list.
  def Update(self):
    if self.tm_last<max(self.fv_data.tm_last_topic[:2]):
      for side in (0,1):
        self.dstate_list[side].append(self.GetDState(side))
        if len(self.dstate_list[side])>self.smoothing_filter_len:
          self.dstate_list[side].pop(0)
          if not self.is_initialized:
            self.dstate0= [self.mean(self.dstate_list[0]), self.mean(self.dstate_list[1])]
            self.is_initialized= True
      self.tm_last= max(self.fv_data.tm_last_topic[:2])

      if self.is_initialized:
        #print self.dstate_list
        #print self.fv_data.dstate_array
        #print self.mean(self.dstate_list[0])-self.dstate0[0], self.mean(self.dstate_list[1])-self.dstate0[1]
        if self.mean(self.dstate_list[0])>=self.dstate0[0]+self.dth or self.mean(self.dstate_list[1])>=self.dstate0[1]+self.dth:
          self.is_detected= True

  #Wait until the initialization is finished.
  def WaitForInitialization(self, dt=0.001, stop_callback=None):
    while not self.IsInitialized():
      if stop_callback is not None and stop_callback():  break
      self.Update()
      rospy.sleep(dt)

  #True if the initialization is finished (data for initial filter values are corrected).
  def IsInitialized(self):
    return self.is_initialized

  #True if force is detected.
  def IsDetected(self):
    return self.is_detected


def Loop(fvg):
  force_detector= TForceChangeDetector(fvg.fv.data, th=fvg.fv_ctrl_param.grasp_th, filter_len=fvg.fv_ctrl_param.grasp_filter_len, dstate_th=fvg.fv_ctrl_param.grasp_dstate_th)
  force_detector.Init()

  #continue_cond= lambda: n_change()<5

  fvg.fv.CallSrv('stop_detect_obj')

  if fvg.gripper.Is('DxlGripper'):
    fvg.gripper.StartHolding()

  try:
    g_pos= fvg.GripperPosition()
    while fvg.script_is_active and not rospy.is_shutdown():
      if g_pos<0.001 or force_detector.IsDetected():
        print 'Done'
        break

      if force_detector.IsInitialized():
        g_pos-= fvg.fv_ctrl_param.min_gstep
        fvg.GripperMoveTo(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort, speed=1.0, blocking=False)
        for i in range(100):
          if abs(fvg.GripperPosition()-g_pos)<0.5*fvg.fv_ctrl_param.min_gstep:  break
          rospy.sleep(0.0001)
        #rospy.sleep(0.1)
        g_pos= fvg.GripperPosition()

      force_detector.Update()

  finally:
    if fvg.gripper.Is('DxlGripper'):
      fvg.gripper.StopHolding()
