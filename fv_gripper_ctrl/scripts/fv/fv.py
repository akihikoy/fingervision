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
  table= {
        'srv_separated':    not isinstance(node_names, str),
        fv_names[RIGHT]:    RIGHT,
        fv_names[LEFT]:     LEFT,
        }
  if table['srv_separated']:
    for srv in sum((srvs for _,srvs in SRV_TABLE),()):
      table[srv+'_r']= '/fingervision/{}/{}'.format(node_names[RIGHT],srv)
      table[srv+'_l']= '/fingervision/{}/{}'.format(node_names[LEFT],srv)
  else:
    for srv in sum((srvs for _,srvs in SRV_TABLE),()):
      table[srv]= '/fingervision/{}/{}'.format(node_names,srv)
  return table

def Filter1Wrench(ct,msg):
  table= ct.cnt.fvconf
  l= ct.cnt.fv
  side= table[msg.fv]
  l.posforce_array[side]= np.array(msg.posforce_array).reshape(len(msg.posforce_array)/5,5).tolist()
  l.force_array[side]= np.array(msg.force_array).reshape(len(msg.force_array)/6,6).tolist()
  l.dstate_array[side]= msg.dstate_array
  l.force[side]= msg.force
  l.dstate[side]= msg.dstate
  l.tm_last_topic[side]= msg.header.stamp

  if ct.callback.fv_wrench[side] is not None:
    ct.callback.fv_wrench[side](ct, l, side)

  #Broadcast the TF of FV.
  lw_xe= ct.cnt.g_param['lx']
  gpos= ct.gripper.Position()
  if gpos is not None:
    lw_xg= Transform(lw_xe,[0,(-0.5*gpos,+0.5*gpos)[side],0, 0,0,0,1])
    ct.br.sendTransform(lw_xg[0:3],lw_xg[3:],
        msg.header.stamp, msg.header.frame_id, ct.cnt.frame_id)

def Filter1ObjInfo(ct,msg):
  table= ct.cnt.fvconf
  l= ct.cnt.fv
  side= table[msg.fv]
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

  if ct.callback.fv_objinfo[side] is not None:
    ct.callback.fv_objinfo[side](ct, l, side)

'''
Start to subscribe topics, setup services.
fv_names: FV camera names (dict, {LEFT/RIGHT:fv_l/fv_r}.
node_names: FV node names (dict, {LEFT/RIGHT:fv_l_node/fv_r_node}),
            or a common FV node name (dict, fv_node),
            or None (node_names==fv_names).
'''
def Setup(ct, fv_names, node_names=None):
  print '''Setup FV:
  fv_names: {fv_names}
  node_names: {node_names}'''.format(fv_names=fv_names,node_names=node_names)

  ct.cnt.fv= TContainer()
  if 'fv_wrench' not in ct.callback:  ct.callback.fv_wrench= TContainer()
  ct.callback.fv_wrench= [None,None]  #Callback in Filter1Wrench
  if 'fv_objinfo' not in ct.callback:  ct.callback.fv_objinfo= TContainer()
  ct.callback.fv_objinfo= [None,None]  #Callback in Filter1ObjInfo
  l= ct.cnt.fv
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

  table= GetFVSrvDict(fv_names,node_names)
  print 'Configured info with:',fv_names,node_names
  ct.cnt.fvconf= table
  if table['srv_separated']:
    for srvtype,srvs in SRV_TABLE:
      for srv in srvs:
        ct.AddSrvP(srv+'_l', table[srv+'_l'], srvtype, persistent=False, time_out=3.0)
        ct.AddSrvP(srv+'_r', table[srv+'_r'], srvtype, persistent=False, time_out=3.0)
  else:
    for srvtype,srvs in SRV_TABLE:
      for srv in srvs:
        ct.AddSrvP(srv, table[srv], srvtype, persistent=False, time_out=3.0)

  if table['srv_separated']:
    ct.srvp['start_detect_obj_r'](std_srvs.srv.EmptyRequest())
    ct.srvp['start_detect_obj_l'](std_srvs.srv.EmptyRequest())
  else:
    ct.srvp['start_detect_obj'](std_srvs.srv.EmptyRequest())

  ct.AddSubW('fv_filter1_wrench', '/fingervision/fv_filter1_wrench', fingervision_msgs.msg.Filter1Wrench, lambda msg:Filter1Wrench(ct,msg), time_out=3.0)
  ct.AddSubW('fv_filter1_objinfo', '/fingervision/fv_filter1_objinfo', fingervision_msgs.msg.Filter1ObjInfo, lambda msg:Filter1ObjInfo(ct,msg), time_out=3.0)

#Stop to subscribe topics.
def Clear(ct):
  print 'Stopping','fv'
  ct.cnt.fv.running= False
  if 'fv_wrench' in ct.callback:  ct.callback.fv_wrench= [None,None]
  if 'fv_objinfo' in ct.callback:  ct.callback.fv_objinfo= [None,None]

  table= ct.cnt.fvconf
  if table is None:  return
  for srv in ('fv_filter1_wrench','fv_filter1_objinfo'):
    ct.DelSub(srv)
  if table['srv_separated']:
    for srv in sum((srvs for _,srvs in SRV_TABLE),()):
      ct.DelSrvP(srv+'_r')
      ct.DelSrvP(srv+'_l')
  else:
    for srv in sum((srvs for _,srvs in SRV_TABLE),()):
      ct.DelSrvP(srv)

#Check if FingerVision is working properly.
def IsActive(ct):
  is_active= False
  is_active= None not in ct.cnt.fv.tm_last_topic \
    and (rospy.Time.now()-min(ct.cnt.fv.tm_last_topic)).to_sec()<0.2
  return is_active

'''
Run a command in ('pause','resume','clear_obj','start_detect_obj','stop_detect_obj').
  'pause'            : Pause video processing.
  'resume'           : Resume video processing.
  'clear_obj'        : Clear detected object models.
  'start_detect_obj' : Start detecting object.
  'stop_detect_obj'  : Stop detecting object.
'''
def CallSrv(ct, command):
  if command in SRV_TABLE[0][1]:
    table= ct.cnt.fvconf
    if table['srv_separated']:
      ct.srvp[command+'_r'](std_srvs.srv.EmptyRequest())
      ct.srvp[command+'_l'](std_srvs.srv.EmptyRequest())
    else:
      ct.srvp[command](std_srvs.srv.EmptyRequest())
  else:
    CPrint(4,'fv.CallSrv: Invalid command:',command)

#Set frame-skip.
#skip: Frames to be skipped. 0: No skip.
def SetFrameSkip(ct, skip):
  set_frame_skip_req= fingervision_msgs.srv.SetInt32Request()
  set_frame_skip_req.data= skip
  table= ct.cnt.fvconf
  if table['srv_separated']:
    ct.srvp['set_frame_skip_r'](set_frame_skip_req)
    ct.srvp['set_frame_skip_l'](set_frame_skip_req)
  else:
    ct.srvp['set_frame_skip'](set_frame_skip_req)

#Take snapshots of current images.
#prefix: Snapshot prefix.
#ext: Extension.
def TakeSnapshot(ct, prefix='/tmp/fv', extension='.png'):
  take_snapshot_req= fingervision_msgs.srv.TakeSnapshotRequest()
  take_snapshot_req.prefix= prefix
  take_snapshot_req.ext= extension
  files= []
  table= ct.cnt.fvconf
  if table['srv_separated']:
    files+= ct.srvp['take_snapshot_r'](take_snapshot_req).files
    files+= ct.srvp['take_snapshot_l'](take_snapshot_req).files
  else:
    files= ct.srvp['take_snapshot'](take_snapshot_req).files
  return files

