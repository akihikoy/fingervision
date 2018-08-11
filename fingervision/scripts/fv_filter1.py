#!/usr/bin/python
#\file    fv_filter1.py
#\brief   Filter the output topics of fv_core_node.
#         BlobMoves: Convert blob_moves to force and torque estimates.
#         ProxVision: Convert prox_vision to object center, orientation, and area.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.10, 2018
import roslib; roslib.load_manifest('ay_trick')
import rospy
from ay_py.core import *
from ay_py.ros import *  # LRToStrS, etc.
roslib.load_manifest('fingervision_msgs')
import fingervision_msgs.msg
import fingervision_msgs.srv
import geometry_msgs.msg

def BlobMoves(msg,fv,side,pub_fwrench,pub_wrench):
  cx= msg.width/2
  cy= msg.height/2
  div= float(msg.width+msg.height)/2.0
  def convert_raw(mv):
    fscale= [1.0,1.0,1.0]
    rxy= ((mv.Pox-cx)/div, (mv.Poy-cy)/div)
    fxy= (mv.DPx, mv.DPy)
    fz= la.norm(fxy) # max(0.0,mv.DS)
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

def ProxVision(msg,fv,pub_fobjinfo):
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

  msg2= fingervision_msgs.msg.Filter1ObjInfo()
  msg2.header= msg.header
  msg2.fv= fv
  msg2.mv_s= mv_s
  msg2.obj_s= obj_s
  msg2.obj_center= obj_center
  msg2.obj_orientation= obj_orientation
  msg2.obj_area= obj_area
  pub_fobjinfo.publish(msg2)

if __name__=='__main__':
  rospy.init_node('fv_filter1')
  fv= 'fv'
  side_str= 'r'
  fv= rospy.get_param('~fv', fv)
  side_str= rospy.get_param('~side', side_str)
  side= StrToLR(side_str)
  if side is None:  side= StrToID(side_str)

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
                           fingervision_msgs.msg.BlobMoves, lambda msg:BlobMoves(msg,fv,side,pub_fwrench,pub_wrench))
  sub_pv= rospy.Subscriber(rospy.get_namespace()+'{fv}/prox_vision'.format(fv=fv),
                           fingervision_msgs.msg.ProxVision, lambda msg:ProxVision(msg,fv,pub_fobjinfo))
  rospy.spin()
