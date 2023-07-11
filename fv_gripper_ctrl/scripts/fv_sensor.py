#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
import tf
import sensor_msgs.msg
import std_srvs.srv
import fingervision_msgs.msg
import fingervision_msgs.srv

SRV_TABLE=[
  (std_srvs.srv.Empty, ('clear_obj','pause','resume','start_detect_obj','stop_detect_obj')),
  (fingervision_msgs.srv.SetInt32, ('set_frame_skip',)),
  (fingervision_msgs.srv.TakeSnapshot, ('take_snapshot',)),
  ]

'''
fv_names: FV camera names (dict, {LEFT/RIGHT:fv_l/fv_r}).
node_names: FV node names (dict, {LEFT/RIGHT:fv_l_node/fv_r_node}), or a single common node name.
    If node_names is None, fv_names is used as node_names.
'''
def GetFVSrvDict(fv_names, node_names=None):
  if node_names is None:  node_names= fv_names
  config= {
        'srv_separated':    not isinstance(node_names, str),
        fv_names[RIGHT]:    RIGHT,
        fv_names[LEFT]:     LEFT,
        }
  if config['srv_separated']:
    for srv in sum((srvs for _,srvs in SRV_TABLE),()):
      config[srv+'_r']= '/fingervision/{}/{}'.format(node_names[RIGHT],srv)
      config[srv+'_l']= '/fingervision/{}/{}'.format(node_names[LEFT],srv)
  else:
    for srv in sum((srvs for _,srvs in SRV_TABLE),()):
      config[srv]= '/fingervision/{}/{}'.format(node_names,srv)
  return config


'''
Utility class to handle FV sensors.
It is assumed that two FV sensors are installed on a gripper.
'''
class TFVSensor(TROSUtil):
  def __init__(self):
    super(TFVSensor,self).__init__()
    self.config= None
    self.data= TContainer()
    self.callback= TContainer()
    self.gripper= None
    self.g_param= None
    self.frame_id= None
    self.br= None

  def __del__(self):
    self.Cleanup()
    if TFVSensor is not None:  super(TFVSensor,self).__del__()
    print 'TFVSensor: done',self

  def Cleanup(self):
    self.Stop()
    for k in self.callback.keys():
      self.callback[k]= None  #We do not delete
    if TFVSensor is not None:  super(TFVSensor,self).Cleanup()

  '''
  Setup the utility.
  gripper: Gripper utility object.
  g_param: Gripper parameters.
  frame_id: Frame ID.
  fv_names, node_names: cf. SetupFV()
  '''
  def Setup(self, gripper, g_param, frame_id, fv_names, node_names=None):
    self.gripper= gripper
    self.g_param= g_param
    self.frame_id= frame_id
    self.br= tf.TransformBroadcaster()
    self.SetupFV(fv_names, node_names)

  '''
  Start to subscribe topics, setup services.
  fv_names: FV camera names (dict, {LEFT/RIGHT:fv_l/fv_r}.
  node_names: FV node names (dict, {LEFT/RIGHT:fv_l_node/fv_r_node}),
              or a common FV node name (dict, fv_node),
              or None (node_names==fv_names).
  '''
  def SetupFV(self, fv_names, node_names=None):
    print '''Setup FV:
    fv_names: {fv_names}
    node_names: {node_names}'''.format(fv_names=fv_names,node_names=node_names)

    self.callback.fv_wrench= [None,None]  #Callbacks in Filter1WrenchCallback
    self.callback.fv_objinfo= [None,None]  #Callbacks in Filter1ObjInfoCallback
    l= self.data
    l.fv= 'fv'
    l.posforce_array= [None,None]
    l.force_array= [None,None]
    l.dstate_array= [None,None]
    l.force= [None,None]
    l.dstate= [0,0]
    l.obj_s= [None,None]
    l.mv_s= [None,None]
    l.obj_center= [None,None]
    l.obj_orientation= [None,None]
    l.obj_area= [None,None]
    l.d_obj_center= [None,None]
    l.d_obj_orientation= [None,None]
    l.d_obj_area= [None,None]
    l.obj_area_filtered= [None,None]
    l.d_obj_center_filtered= [None,None]
    l.d_obj_orientation_filtered= [None,None]
    l.d_obj_area_filtered= [None,None]
    l.tm_last_topic= [None,None,None,None]  #ros time of last topic receiving. wrench-r,l, objinfo-r,l
    l.running= True

    self.config= GetFVSrvDict(fv_names,node_names)
    print 'Configured info with:',fv_names,node_names
    if self.config['srv_separated']:
      for srvtype,srvs in SRV_TABLE:
        for srv in srvs:
          self.AddSrvP(srv+'_l', self.config[srv+'_l'], srvtype, persistent=False, time_out=3.0)
          self.AddSrvP(srv+'_r', self.config[srv+'_r'], srvtype, persistent=False, time_out=3.0)
    else:
      for srvtype,srvs in SRV_TABLE:
        for srv in srvs:
          self.AddSrvP(srv, self.config[srv], srvtype, persistent=False, time_out=3.0)

    if self.config['srv_separated']:
      self.srvp['start_detect_obj_r'](std_srvs.srv.EmptyRequest())
      self.srvp['start_detect_obj_l'](std_srvs.srv.EmptyRequest())
    else:
      self.srvp['start_detect_obj'](std_srvs.srv.EmptyRequest())

    self.AddSub('fv_filter1_wrench', '/fingervision/fv_filter1_wrench', fingervision_msgs.msg.Filter1Wrench, self.Filter1WrenchCallback)
    self.AddSub('fv_filter1_objinfo', '/fingervision/fv_filter1_objinfo', fingervision_msgs.msg.Filter1ObjInfo, self.Filter1ObjInfoCallback)

  #Stop subscribing topics.
  def Stop(self):
    print 'Stopping','fv'
    self.data.running= False
    if 'fv_wrench' in self.callback:  self.callback.fv_wrench= [None,None]
    if 'fv_objinfo' in self.callback:  self.callback.fv_objinfo= [None,None]

    if self.config is None:  return
    for srv in ('fv_filter1_wrench','fv_filter1_objinfo'):
      self.DelSub(srv)
    if self.config['srv_separated']:
      for srv in sum((srvs for _,srvs in SRV_TABLE),()):
        self.DelSrvP(srv+'_r')
        self.DelSrvP(srv+'_l')
    else:
      for srv in sum((srvs for _,srvs in SRV_TABLE),()):
        self.DelSrvP(srv)

  def Filter1WrenchCallback(self, msg):
    side= self.config[msg.fv]
    l= self.data
    l.posforce_array[side]= np.array(msg.posforce_array).reshape(len(msg.posforce_array)/5,5).tolist()
    l.force_array[side]= np.array(msg.force_array).reshape(len(msg.force_array)/6,6).tolist()
    l.dstate_array[side]= msg.dstate_array
    l.force[side]= msg.force
    l.dstate[side]= msg.dstate
    l.tm_last_topic[side]= msg.header.stamp

    if self.callback.fv_wrench[side] is not None:
      self.callback.fv_wrench[side](self, l, side)

    #Broadcast the TF of FV.
    lw_xe= self.g_param['lx']
    gpos= self.gripper.Position()
    if gpos is not None:
      lw_xg= Transform(lw_xe,[0,(-0.5*gpos,+0.5*gpos)[side],0, 0,0,0,1])
      self.br.sendTransform(lw_xg[0:3],lw_xg[3:],
          msg.header.stamp, msg.header.frame_id, self.frame_id)

  def Filter1ObjInfoCallback(self, msg):
    side= self.config[msg.fv]
    l= self.data
    l.obj_s[side]= msg.obj_s
    l.mv_s[side]= msg.mv_s
    l.obj_center[side]= msg.obj_center
    l.obj_orientation[side]= msg.obj_orientation
    l.obj_area[side]= msg.obj_area
    l.d_obj_center[side]= msg.d_obj_center
    l.d_obj_orientation[side]= msg.d_obj_orientation
    l.d_obj_area[side]= msg.d_obj_area
    l.obj_area_filtered[side]= msg.obj_area_filtered
    l.d_obj_center_filtered[side]= msg.d_obj_center_filtered
    l.d_obj_orientation_filtered[side]= msg.d_obj_orientation_filtered
    l.d_obj_area_filtered[side]= msg.d_obj_area_filtered
    l.tm_last_topic[2+side]= msg.header.stamp

    if self.callback.fv_objinfo[side] is not None:
      self.callback.fv_objinfo[side](self, l, side)

  #Check if FingerVision is working properly.
  def IsActive(self):
    is_active= False
    is_active= None not in self.data.tm_last_topic \
      and (rospy.Time.now()-min(self.data.tm_last_topic)).to_sec()<0.2
    return is_active

  '''
  Run a command in ('pause','resume','clear_obj','start_detect_obj','stop_detect_obj').
    'pause'            : Pause video processing.
    'resume'           : Resume video processing.
    'clear_obj'        : Clear detected object models.
    'start_detect_obj' : Start detecting object.
    'stop_detect_obj'  : Stop detecting object.
  '''
  def CallSrv(self, command):
    if command in SRV_TABLE[0][1]:
      if self.config['srv_separated']:
        self.srvp[command+'_r'](std_srvs.srv.EmptyRequest())
        self.srvp[command+'_l'](std_srvs.srv.EmptyRequest())
      else:
        self.srvp[command](std_srvs.srv.EmptyRequest())
    else:
      CPrint(4,'fv.CallSrv: Invalid command:',command)

  #Set frame-skip.
  #skip: Frames to be skipped. 0: No skip.
  def SetFrameSkip(self, skip):
    set_frame_skip_req= fingervision_msgs.srv.SetInt32Request()
    set_frame_skip_req.data= skip
    if self.config['srv_separated']:
      self.srvp['set_frame_skip_r'](set_frame_skip_req)
      self.srvp['set_frame_skip_l'](set_frame_skip_req)
    else:
      self.srvp['set_frame_skip'](set_frame_skip_req)

  #Take snapshots of current images.
  #prefix: Snapshot prefix.
  #ext: Extension.
  def TakeSnapshot(self, prefix='/tmp/fv', extension='.png'):
    take_snapshot_req= fingervision_msgs.srv.TakeSnapshotRequest()
    take_snapshot_req.prefix= prefix
    take_snapshot_req.ext= extension
    files= []
    if self.config['srv_separated']:
      files+= self.srvp['take_snapshot_r'](take_snapshot_req).files
      files+= self.srvp['take_snapshot_l'](take_snapshot_req).files
    else:
      files= self.srvp['take_snapshot'](take_snapshot_req).files
    return files

