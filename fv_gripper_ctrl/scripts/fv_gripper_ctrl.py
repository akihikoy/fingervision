#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
import tf
import sensor_msgs.msg
import fv.fv
import fv.grasp
import fv.hold
import fv.inhand
import fv.open
import fv.openif


class TCoreToolMini(TROSUtil):
  def __init__(self):
    super(TCoreToolMini,self).__init__()
    self.gripper= None
    self.cnt= TContainer()
    self.viz= TContainer()
    self.thread_manager= TThreadManager()
    self.callback= TContainer()
    self.br= tf.TransformBroadcaster()

  def __del__(self):
    self.Cleanup()
    super(TCoreToolMini,self).__del__()
    print 'TCoreToolMini: done',self

  def Cleanup(self):
    self.thread_manager.StopAll()
    if self.gripper is not None:
      self.gripper.Cleanup()
      del self.gripper
      self.gripper= None
    for k in self.callback.keys():
      self.callback[k]= None  #We do not delete
    super(TCoreToolMini,self).Cleanup()

def CreateGripper(gripper_type, gripper_node='gripper_driver', is_sim=False):
  gripper= None
  param= None
  if gripper_type in ('RHP12RNGripper','RHP12RNAGripper'):
    if not is_sim:
      mod= __import__('ay_py.ros.rbt_rhp12rn',globals(),None,('TRHP12RNGripper',))
      gripper= mod.TRHP12RNGripper(node_name=gripper_node)
    else:
      gripper= TSimGripper2F1(('RHP12RNGripper','ThGripper'),pos_range=[0.0,0.109])
    param={
      'lx': [0.0,0.0,0.218, 0.5,-0.5,0.5,0.5],
      'bound_box':{
        'dim': [0.16,0.04,0.218],
        'center': [0.0,0.0,0.109, 0.0,0.0,0.0,1.0],
        }
      }
  elif gripper_type=='DxlGripper':
    if not is_sim:
      mod= __import__('ay_py.ros.rbt_dxlg',globals(),None,('TDxlGripper',))
      gripper= mod.TDxlGripper(node_name=gripper_node)
    else:
      gripper= TSimGripper2F1((gripper_type,),pos_range=[0.0,0.095])
    param={
      'lx': [0.0,0.0,0.16, 0.5,-0.5,0.5,0.5],
      'bound_box':{
        'dim': [0.08,0.02,0.16],
        'center': [0.0,0.0,0.08, 0.0,0.0,0.0,1.0],
        }
      }
  elif gripper_type=='EZGripper':
    if not is_sim:
      mod= __import__('ay_py.ros.rbt_ezg',globals(),None,('TEZGripper',))
      gripper= mod.TEZGripper(node_name=gripper_node)
    else:
      gripper= TSimGripper2F1((gripper_type,),pos_range=[0.0,0.150])
    param={
      'lx': [0,0,0, 0.0,-0.70710678,0.0,0.70710678],
      'bound_box':{
        'dim': [0.25,0.20,0.045],
        'center': [-0.125,0.0,0.0, 0.0,0.0,0.0,1.0],
        }
      }
  elif gripper_type.startswith('DxlpO2Gripper'):
    finger_type= gripper_type.replace('DxlpO2Gripper_','')
    fts= {'Straight1':'st1','SRound1':'sr1','Fork1':'f1'}[finger_type]
    if not is_sim:
      mod= __import__('ay_py.ros.rbt_dxlpo2',globals(),None,('TDxlpO2Gripper',))
      gripper= mod.TDxlpO2Gripper(node_name=gripper_node, finger_type=finger_type)
    else:
      gripper= TSimGripper2F1((gripper_type,),pos_range={'st1':[0.0,0.300],'sr1':[0.0,0.1950],'f1':[-0.0189,0.200]}[fts])
    param={
      'lx': [0.0,0.0,{'st1':0.31,'sr1':0.22,'f1':0.22}[fts], 0.5,-0.5,0.5,0.5],
      'bound_box':{
        'dim': [0.30,0.154,{'st1':0.31,'sr1':0.22,'f1':0.22}[fts]],
        'center': [0.0,0.0,{'st1':0.155,'sr1':0.11,'f1':0.11}[fts], 0.0,0.0,0.0,1.0],
        }
      }
  elif gripper_type=='DxlpY1Gripper':
    if not is_sim:
      mod= __import__('ay_py.ros.rbt_dxlpy1',globals(),None,('TDxlpY1Gripper',))
      gripper= mod.TDxlpY1Gripper(node_name=gripper_node)
    else:
      gripper= TSimGripper2F1((gripper_type,),pos_range=[0.0,0.133])
    param={
      'lx': [0.0,0.0,0.264, 0.5,-0.5,0.5,0.5],
      'bound_box':{
        'dim': [0.115,0.120,0.264],
        'center': [0.0,0.0,0.132, 0.0,0.0,0.0,1.0],
        }
      }
  return gripper, param

def JoyCallback(state, steps, wsteps, gsteps, data):
  gsteps[0]= 0.0
  steps[:]= [0.0]*3
  wsteps[:]= [0.0]*3
  state[1]= 'no_cmd'
  state[3]= False

  #multiplier= 1.0
  multiplier= (1.0+data.axes[5])*0.5 + (1.0-data.axes[2])*2.0  #RT, LT
  #if data.buttons[1] > 0:
    #multiplier= 2.0 * multiplier

  axes= [(ax if abs(ax)>0.15 else 0.0) for ax in data.axes]

  if data.buttons[7] > 0:  #START
    state[0]= 'quit'
    return

  if data.buttons[1] > 0:  #B
    state[1]= 'cmd_B'
    #return
  if data.buttons[2] > 0:  #X
    state[1]= 'cmd_X'
    #return

  if data.buttons[3] > 0:  #Y
    state[1]= 'cmd_Y'
    #return

  dpad_btn= [1 if btn>0 else 0 for btn in data.buttons[11:15]]
  if any(dpad_btn):
    if sum(dpad_btn)==1:  #Need to press exactly one d-pad button at the same time
      if data.buttons[11] > 0:  #LEFT
        state[1]= 'cmd_left'
        #return
      if data.buttons[12] > 0:  #RIGHT
        state[1]= 'cmd_right'
        #return
      if data.buttons[13] > 0:  #UP
        state[1]= 'cmd_up'
        #return
      if data.buttons[14] > 0:  #DOWN
        state[1]= 'cmd_down'
        #return

  if data.buttons[0] > 0:  #A
    state[1]= 'grip'
    gsteps[0]= multiplier * axes[0]
    #return

  if data.buttons[5] <= 0:  #not RB
    state[3]= False
  else:
    state[3]= True

def CtrlLoop(ct):
  if not any((ct.gripper.Is('RobotiqNB'),ct.gripper.Is('DxlGripper'),ct.gripper.Is('RHP12RNGripper'),ct.gripper.Is('EZGripper'),ct.gripper.Is('DxlpO2Gripper'),ct.gripper.Is('DxlO3Gripper'))):
    CPrint(4,'This program works only with RobotiqNB, DxlGripper, RHP12RNGripper, EZGripper, DxlpO2Gripper, and DxlO3Gripper.')
    return

  if ct.gripper.Is('DxlGripper'):
    active_holding= [False]

  steps= [0.0, 0.0, 0.0]
  wsteps= [0.0, 0.0, 0.0]
  gsteps= [0.0]
  state= ['run', 'no_cmd', None, False]  #run/quit, no_cmd/CMD, ARM, ACTIVE_BTN

  gstate_range= ct.gripper.PosRange()
  gstate= ct.gripper.Position() if ct.gripper.IsInitialized else 0.0
  if ct.gripper.IsInitialized:
    ct.gripper.Move(gstate)

  ct.AddSub('joy', 'joy', sensor_msgs.msg.Joy, lambda msg: JoyCallback(state, steps, wsteps, gsteps, msg))

  kbhit= TKBHit()
  try:
    while state[0]=='run' and not rospy.is_shutdown():
      if kbhit.IsActive():
        key= kbhit.KBHit()
        if key=='q':
          break;
        elif key is not None:
          state[1]= 'key_'+str(key)
      else:
        break

      if state[1]=='key_i' or state[1]=='cmd_Y':
        if 'vs_inhand' not in ct.thread_manager.thread_list:
          fv.inhand.On(ct)
        else:
          fv.inhand.Off(ct)
        state[1]= 'no_cmd'
      elif state[1]=='cmd_left':
        fv.open.Run(ct)
        state[1]= 'no_cmd'
      elif state[1]=='cmd_right':
        if state[3]:
          fv.grasp.On(ct)
        else:
          fv.grasp.Off(ct)
        state[1]= 'no_cmd'
      elif state[1]=='cmd_up':
        if state[3]:
          fv.hold.On(ct)
        else:
          fv.hold.Off(ct)
        state[1]= 'no_cmd'
      elif state[1]=='cmd_down':
        if state[3]:
          fv.openif.On(ct)
        else:
          fv.openif.Off(ct)
        state[1]= 'no_cmd'

      elif state[1]=='key_[':
        gsteps[0]= 0.005
        state[1]= 'grip'
      elif state[1]=='key_]':
        gsteps[0]= -0.005
        state[1]= 'grip'
      elif state[1]=='key_{':
        gsteps[0]= 0.01
        state[1]= 'grip'
      elif state[1]=='key_}':
        gsteps[0]= -0.01
        state[1]= 'grip'

      if state[1]=='grip':
        if ct.gripper.Is('DxlGripper'):
          if not active_holding[0]:
            ct.gripper.StartHolding()
            active_holding[0]= True
        gstate= ct.gripper.Position() + 0.005*gsteps[0]
        if gstate<gstate_range[0]:  gstate= gstate_range[0]
        if gstate>gstate_range[1]:  gstate= gstate_range[1]
        ct.gripper.Move(gstate,max_effort=100.0,speed=100.0)

      if not state[1]=='grip':
        if ct.gripper.Is('DxlGripper'):
          if active_holding[0]:
            ct.gripper.StopHolding()
            active_holding[0]= False

      VizGripper(ct)
      rospy.sleep(0.005)

  finally:
    kbhit.Deactivate()
    ct.DelSub('joy')
    if ct.gripper.Is('DxlGripper'):
      if active_holding[0]:
        ct.gripper.StopHolding()
        active_holding[0]= False
    print 'Finished'

def VizGripper(ct):
  xw= [0,0,0, 0,0,0,1]
  viz= ct.viz.gripper
  mid= 0
  mid= viz.AddCoord(xw, scale=[0.03,0.002], alpha=1.0, mid=mid)
  lw_xe= ct.cnt.g_param['lx']
  bb_dim= ct.cnt.g_param['bound_box']['dim']
  bb_center= ct.cnt.g_param['bound_box']['center']
  mid= viz.AddCube(Transform(xw,bb_center), bb_dim, rgb=viz.ICol(3), alpha=0.5, mid=mid)
  #Visualize finger pads:
  gpos= ct.gripper.Position()
  lw_xgl= Transform(lw_xe,[0,+0.5*gpos,0, 0,0,0,1])
  lw_xgr= Transform(lw_xe,[0,-0.5*gpos,0, 0,0,0,1])
  mid= viz.AddCube(Transform(xw,lw_xgl), [0.015,0.003,0.03], rgb=viz.ICol(1), alpha=0.8, mid=mid)
  mid= viz.AddCube(Transform(xw,lw_xgr), [0.015,0.003,0.03], rgb=viz.ICol(1), alpha=0.8, mid=mid)
  mid= viz.AddCoord(Transform(xw,lw_xe), scale=[0.01,0.001], alpha=1.0, mid=mid)
  viz.Publish()

if __name__ == '__main__':
  rospy.init_node('fv_gripper_ctrl')

  gripper_type= rospy.get_param('~gripper_type', 'RHP12RNGripper')
  gripper_node= rospy.get_param('~gripper_node', 'gripper_driver')
  is_sim= rospy.get_param('~is_sim', False)
  fv_names= rospy.get_param('~fv_names', {RIGHT:'fvp_1_r',LEFT:'fvp_1_l'})
  fv_nodes= rospy.get_param('~fv_nodes', None)

  ct= TCoreToolMini()
  ct.cnt.frame_id= 'base_link'
  ct.gripper,ct.cnt.g_param= CreateGripper(gripper_type, gripper_node=gripper_node, is_sim=is_sim)
  fv.fv.Setup(ct, fv_names=fv_names, node_names=fv_nodes)
  ct.viz.gripper= TSimpleVisualizerArray(rospy.Duration(1.0), name_space='visualizer_viz', frame=ct.cnt.frame_id)

  CtrlLoop(ct)
  ct.Cleanup()

