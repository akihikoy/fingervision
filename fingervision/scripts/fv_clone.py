#!/usr/bin/python
#\file    fv_clone.py
#\brief   Creating a clone of fv_core_node.
#         This script is to be used for a system with small CPU power
#         where running fv_core_node for two FVs is hard.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.04, 2018
import roslib
import rospy
roslib.load_manifest('fingervision_msgs')
import fingervision_msgs.msg
import fingervision_msgs.srv
import std_srvs.srv

def ModifyFV(msg,fv_to):
  msg.header.frame_id= fv_to
  msg.camera_name= fv_to
  return msg

if __name__=='__main__':
  rospy.init_node('fv_clone')
  fv_from= 'fv_l'
  fv_to= 'fv_r'
  fv_from= rospy.get_param('~fv_from', fv_from)
  fv_to= rospy.get_param('~fv_to', fv_to)

  pub_bm= rospy.Publisher(rospy.get_namespace()+'{fv}/blob_moves'.format(fv=fv_to),
                          fingervision_msgs.msg.BlobMoves, queue_size=10)
  pub_pv= rospy.Publisher(rospy.get_namespace()+'{fv}/prox_vision'.format(fv=fv_to),
                          fingervision_msgs.msg.ProxVision, queue_size=10)

  sub_bm= rospy.Subscriber(rospy.get_namespace()+'{fv}/blob_moves'.format(fv=fv_from),
                           fingervision_msgs.msg.BlobMoves, lambda msg:pub_bm.publish(ModifyFV(msg,fv_to)))
  sub_pv= rospy.Subscriber(rospy.get_namespace()+'{fv}/prox_vision'.format(fv=fv_from),
                           fingervision_msgs.msg.ProxVision, lambda msg:pub_pv.publish(ModifyFV(msg,fv_to)))


  srvp_pause= rospy.ServiceProxy('/fingervision/{fv}/pause'.format(fv=fv_from),
                                     std_srvs.srv.Empty)
  srvp_resume= rospy.ServiceProxy('/fingervision/{fv}/resume'.format(fv=fv_from),
                                     std_srvs.srv.Empty)
  srvp_clear_obj= rospy.ServiceProxy('/fingervision/{fv}/clear_obj'.format(fv=fv_from),
                                     std_srvs.srv.Empty)
  srvp_set_frame_skip= rospy.ServiceProxy('/fingervision/{fv}/set_frame_skip'.format(fv=fv_from),
                                          fingervision_msgs.srv.SetInt32)
  srvp_start_detect_obj= rospy.ServiceProxy('/fingervision/{fv}/start_detect_obj'.format(fv=fv_from),
                                            std_srvs.srv.Empty)
  srvp_stop_detect_obj= rospy.ServiceProxy('/fingervision/{fv}/stop_detect_obj'.format(fv=fv_from),
                                            std_srvs.srv.Empty)

  srv_pause= rospy.Service('/fingervision/{fv}/pause'.format(fv=fv_to),
                           std_srvs.srv.Empty, lambda req:srvp_pause(req))
  srv_resume= rospy.Service('/fingervision/{fv}/resume'.format(fv=fv_to),
                            std_srvs.srv.Empty, lambda req:srvp_resume(req))
  srv_clear_obj= rospy.Service('/fingervision/{fv}/clear_obj'.format(fv=fv_to),
                               std_srvs.srv.Empty, lambda req:srvp_clear_obj(req))
  srv_set_frame_skip= rospy.Service('/fingervision/{fv}/set_frame_skip'.format(fv=fv_to),
                                    fingervision_msgs.srv.SetInt32, lambda req:srvp_set_frame_skip(req))
  srv_start_detect_obj= rospy.Service('/fingervision/{fv}/start_detect_obj'.format(fv=fv_to),
                                      std_srvs.srv.Empty, lambda req:srvp_start_detect_obj(req))
  srv_stop_detect_obj= rospy.Service('/fingervision/{fv}/stop_detect_obj'.format(fv=fv_to),
                                     std_srvs.srv.Empty, lambda req:srvp_stop_detect_obj(req))

  rospy.spin()
