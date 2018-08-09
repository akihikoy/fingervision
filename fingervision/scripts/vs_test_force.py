#!/usr/bin/python
#\file    vs_test_force.py
#\brief   Test code of FingerVision (force estimation from marker (blob) movement)
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Apr.18, 2017
import roslib; roslib.load_manifest('lfd_vision')
import rospy
import lfd_vision.msg
import math
import numpy as np
import numpy.linalg as la

def FListToStr(flist, fmt='%0.2f'):
  return '['+' '.join(map(lambda f:fmt%f, flist))+']'

def BlobMoves(msg):
  cx= msg.width/2
  cy= msg.height/2
  div= float(msg.width+msg.height)/2.0
  #Convert marker movement to marker position (p) and 3D force (f)
  def convert_raw(mv):
    fscale= [1.0,1.0,1.0]
    rxy= ((mv.Pox-cx)/div, (mv.Poy-cy)/div)
    fxy= (mv.DPx, mv.DPy)
    fz= la.norm(fxy) # max(0.0,mv.DS)
    #f= [+(fxy[0]*fscale[0]), -(fz*fscale[2]), -(fxy[1]*fscale[1])]
    #p= [+rxy[0], -rxy[1]]
    f= [-(fxy[0]*fscale[0]), +(fz*fscale[2]), -(fxy[1]*fscale[1])]
    p= [-rxy[0], -rxy[1]]
    return p+f
  #Convert position (p) and 3D force (f) to wrench (force + torque)
  def convert_wrench(p_f):
    p,f= p_f[:2], p_f[2:]
    tscale= 1.0
    tau= np.cross([p[0],0.0,p[1]],f)*tscale
    return f+tau.tolist()
  posforce_array= [convert_raw(mv) for mv in msg.data]
  wrench_array= [convert_wrench(p_f) for p_f in posforce_array]
  #Average wrench
  wrench_avr= [sum([wrench[d] for wrench in wrench_array])/float(len(wrench_array)) for d in xrange(6)]
  #Display
  print 'Wrench(average)=', FListToStr(wrench_avr)

if __name__=='__main__':
  rospy.init_node('vs_test_force')
  #Subscribing blob movement topic (marker tracking)
  sub_bm= rospy.Subscriber("/visual_skin_node_ay1/blob_moves_usbcam2fay12_l",
                           lfd_vision.msg.BlobMoves, BlobMoves)
  rospy.spin()
