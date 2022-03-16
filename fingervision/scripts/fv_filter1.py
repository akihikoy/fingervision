#!/usr/bin/python
#\file    fv_filter1.py
#\brief   Filter the output topics of fv_core_node.
#         BlobMoves: Convert blob_moves to force and torque estimates.
#         ProxVision: Convert prox_vision to object center, orientation, and area.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.10, 2018
#\version 0.2
#\date    Jul.3, 2019
#         Added filters to get velocities of object, filtered velocities, etc.
import roslib; roslib.load_manifest('ay_py')
import rospy
from ay_py.core import *
from ay_py.ros import *  # LRToStrS, etc.
roslib.load_manifest('fingervision_msgs')
import fingervision_msgs.msg
import fingervision_msgs.srv
import geometry_msgs.msg

def BlobMoves(msg,fv,side,pub_fwrench,pub_wrench,options):
  cx= msg.width/2
  cy= msg.height/2
  div= float(msg.width+msg.height)/2.0
  def convert_raw(mv):
    fscale= [1.0,1.0,1.0]
    rxy= ((mv.Pox-cx)/div, (mv.Poy-cy)/div)
    fxy= (mv.DPx, mv.DPy)
    fz= la.norm(fxy) if options['normal_f_mode']=='xy_norm' else (max(0.0,mv.DS) if options['normal_f_mode']=='marker_size' else None)
    if side==RIGHT:
      f= [+(fxy[0]*fscale[0]), -(fz*fscale[2]), -(fxy[1]*fscale[1])]
      p= [+rxy[0], -rxy[1]]
    elif side==LEFT:
      f= [-(fxy[0]*fscale[0]), +(fz*fscale[2]), -(fxy[1]*fscale[1])]
      p= [-rxy[0], -rxy[1]]
    else:
      raise NotImplemented('fv_filter1.BlobMoves: side not in (0,1) is not considered.')
    return p+f
  def convert_wrench(p_f):
    p,f= p_f[:2], p_f[2:]
    tscale= 1.0
    tau= np.cross([p[0],0.0,p[1]],f)*tscale
    return f+tau.tolist()
  def convert_dstate(p_f):
    p,f= p_f[:2], p_f[2:]
    fz= abs(f[1])
    if fz<0.8:  dstate= 0
    elif fz<1.8:  dstate= 1
    elif fz<2.5:  dstate= 3
    else:  dstate= 5
    return dstate
  posforce_array= [convert_raw(mv) for mv in msg.data]
  force_array= [convert_wrench(p_f) for p_f in posforce_array]
  dstate_array= [convert_dstate(p_f) for p_f in posforce_array]
  if len(force_array)>0:
    force= [sum([force[d] for force in force_array])/float(len(force_array)) for d in xrange(6)]
  else:
    force= []
  dstate= sum(dstate_array)

  msg2= fingervision_msgs.msg.Filter1Wrench()
  msg2.header= msg.header
  msg2.fv= fv
  msg2.posforce_array= np.array(posforce_array).ravel()  #Serialized
  msg2.force_array= np.array(force_array).ravel()  #Serialized
  msg2.dstate_array= dstate_array
  msg2.force= force
  msg2.dstate= dstate
  pub_fwrench.publish(msg2)

  msg3= geometry_msgs.msg.WrenchStamped()
  msg3.header= msg.header
  if len(force)==6:
    VecToXYZ(force[:3], msg3.wrench.force)
    VecToXYZ(force[3:], msg3.wrench.torque)
  pub_wrench.publish(msg3)

def MovingAverageFilter(value,log,length):
  log.append(value)
  if len(log)>length:  log.pop(0)
  return np.mean(log,0)

def ProxVision(msg,fv,pub_fobjinfo,options,state):
  cx= msg.width/2
  cy= msg.height/2
  div= float(msg.width+msg.height)/2.0
  diva= float(msg.width*msg.height)
  #Object and movement detection (shrunken form, e.g. 3x3 mat)
  obj_s= [i/diva for i in msg.ObjS]  #FIXME: Do not divide by diva since MvS elements are already normalized by the local area!!! 255 would be better
  mv_s= [100.0*i/diva for i in msg.MvS]  #FIXME: Do not divide by diva since MvS elements are already normalized by the local area!!! 255 would be better
  #Get object center and orientation from moment
  m00,m10,m01= msg.ObjM_m[:3]
  mu20,mu11,mu02= msg.ObjM_mu[:3]
  if m00>0.0:
    obj_center= [(m10/m00-cx)/div, (m01/m00-cy)/div]
    obj_orientation= 0.5*math.atan2(2.0*mu11/m00, (mu20/m00-mu02/m00))
    obj_area= m00/diva
  else:
    obj_center= [0.0, 0.0]
    obj_orientation= 0.0
    obj_area= 0.0

  #Temporal filters:
  if 'state_initialized' not in state:
    state.state_initialized= True
    state.last_tm= msg.header.stamp
    state.last_obj_center= obj_center
    state.last_obj_orientation= obj_orientation
    state.last_obj_area= obj_area
    state.obj_area_log= []
    state.d_obj_center_log= []
    state.d_obj_orientation_log= []
    state.d_obj_area_log= []

  dt= (msg.header.stamp-state.last_tm).to_sec()
  if dt>0.0:
    angle_mod= lambda q: Mod(q+0.5*math.pi,math.pi)-0.5*math.pi
    d_obj_center= [(state.last_obj_center[i]-obj_center[i])/dt for i in (0,1)]
    d_obj_orientation= angle_mod(state.last_obj_orientation-obj_orientation)/dt
    d_obj_area= (state.last_obj_area-obj_area)/dt
  else:
    d_obj_center= [0.0, 0.0]
    d_obj_orientation= 0.0
    d_obj_area= 0.0

  state.last_tm= msg.header.stamp
  state.last_obj_center= obj_center
  state.last_obj_orientation= obj_orientation
  state.last_obj_area= obj_area

  obj_area_filtered= MovingAverageFilter(obj_area,state.obj_area_log,options['filter_len'])
  d_obj_center_filtered= MovingAverageFilter(d_obj_center,state.d_obj_center_log,options['filter_len'])
  d_obj_orientation_filtered= MovingAverageFilter(d_obj_orientation,state.d_obj_orientation_log,options['filter_len'])
  d_obj_area_filtered= MovingAverageFilter(d_obj_area,state.d_obj_area_log,options['filter_len'])

  msg2= fingervision_msgs.msg.Filter1ObjInfo()
  msg2.header= msg.header
  msg2.fv= fv
  msg2.mv_s= mv_s
  msg2.obj_s= obj_s
  msg2.obj_center= obj_center
  msg2.obj_orientation= obj_orientation
  msg2.obj_area= obj_area
  msg2.d_obj_center= d_obj_center
  msg2.d_obj_orientation= d_obj_orientation
  msg2.d_obj_area= d_obj_area
  msg2.obj_area_filtered= obj_area_filtered
  msg2.d_obj_center_filtered= d_obj_center_filtered
  msg2.d_obj_orientation_filtered= d_obj_orientation_filtered
  msg2.d_obj_area_filtered= d_obj_area_filtered
  pub_fobjinfo.publish(msg2)

if __name__=='__main__':
  options={
    'normal_f_mode': 'marker_size',  #Options: 'xy_norm','marker_size'
    'filter_len': 5,
    }

  rospy.init_node('fv_filter1')
  fv= 'fv'
  side_str= 'r'
  fv= rospy.get_param('~fv', fv)
  side_str= rospy.get_param('~side', side_str)
  options['normal_f_mode']= rospy.get_param('~normal_f_mode', options['normal_f_mode'])
  options['filter_len']= rospy.get_param('~filter_len', options['filter_len'])
  side= StrToLR(side_str)
  if side is None:  side= StrToID(side_str)

  print '''FV-Filter {node}
    FV: {fv}
    Side: {side}
    Options: {options}'''.format(node=rospy.get_name(),fv=fv,side=side,options=options)

  state_fobjinfo= TContainer(debug=True)

  #Filtered wrench:
  pub_fwrench= rospy.Publisher(rospy.get_namespace()+'fv_filter1_wrench',
                               fingervision_msgs.msg.Filter1Wrench, queue_size=10)
  #Average wrench:
  pub_wrench= rospy.Publisher(rospy.get_namespace()+'{fv}/wrench'.format(fv=fv),
                              geometry_msgs.msg.WrenchStamped, queue_size=10)
  #Filtered object info:
  pub_fobjinfo= rospy.Publisher(rospy.get_namespace()+'fv_filter1_objinfo',
                                fingervision_msgs.msg.Filter1ObjInfo, queue_size=10)

  sub_bm= rospy.Subscriber(rospy.get_namespace()+'{fv}/blob_moves'.format(fv=fv),
                           fingervision_msgs.msg.BlobMoves, lambda msg:BlobMoves(msg,fv,side,pub_fwrench,pub_wrench,options))
  sub_pv= rospy.Subscriber(rospy.get_namespace()+'{fv}/prox_vision'.format(fv=fv),
                           fingervision_msgs.msg.ProxVision, lambda msg:ProxVision(msg,fv,pub_fobjinfo,options,state_fobjinfo))
  rospy.spin()
