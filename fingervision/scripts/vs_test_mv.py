#!/usr/bin/python
#\file    vs_test_mv.py
#\brief   Test code of FingerVision (object and movement detection from proximity vision)
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

def ProxVision(msg):
  cx= msg.width/2
  cy= msg.height/2
  div= float(msg.width+msg.height)/2.0
  diva= float(msg.width*msg.height)
  #Object detection (shrunken to 3x3 mat)
  obj_s= [i/255.0 for i in msg.ObjS]
  #Movement detection (shrunken to 3x3 mat)
  mv_s= [100.0*i/255.0 for i in msg.MvS]
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
  #Display
  print '---'
  print 'Object=', FListToStr(obj_s)
  print 'Movement=', FListToStr(mv_s)
  print 'Object center, orientation, area=',
  print      FListToStr(obj_center), '%0.2f'%(180.0*obj_orientation/math.pi), '%0.2f'%obj_area

if __name__=='__main__':
  rospy.init_node('vs_test_mv')
  #Subscribing proximity vision (object detection, slip detection)
  sub_pv= rospy.Subscriber("/visual_skin_node_ay1/prox_vision_usbcam2fay12_l",
                           lfd_vision.msg.ProxVision, ProxVision)
  rospy.spin()
