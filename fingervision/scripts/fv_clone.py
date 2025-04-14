#!/usr/bin/python3
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

  pub,sub,srvp,srv= {},{},{},{}

  #Clone a publisher of fv_from for fv_to.
  def clone_pub(topic, msg_type, queue_size=10):
    pub[topic]= rospy.Publisher(rospy.get_namespace()+'{fv}/{topic}'.format(fv=fv_to, topic=topic),
                                msg_type, queue_size=queue_size)
    sub[topic]= rospy.Subscriber(rospy.get_namespace()+'{fv}/{topic}'.format(fv=fv_from, topic=topic),
                                 msg_type, lambda msg:pub[topic].publish(ModifyFV(msg,fv_to)))

  clone_pub('blob_moves',  fingervision_msgs.msg.BlobMoves)
  clone_pub('prox_vision', fingervision_msgs.msg.ProxVision)

  #Clone a service of fv_from for fv_to.
  #  dummy: If True, a similar service is created for fv_to but the functionality is removed.
  def clone_srv(service, srv_type, dummy=False):
    srvp[service]= rospy.ServiceProxy('/fingervision/{fv}/{service}'.format(fv=fv_from, service=service),
                                      srv_type)
    if not dummy:
      srv[service]= rospy.Service('/fingervision/{fv}/{service}'.format(fv=fv_to, service=service),
                                  srv_type, lambda req:srvp[service](req))
    else:
      srv[service]= rospy.Service('/fingervision/{fv}/{service}'.format(fv=fv_to, service=service),
                                  srv_type, lambda req:srv_type._response_class())

  clone_srv('pause'            , std_srvs.srv.Empty, dummy=True)
  clone_srv('resume'           , std_srvs.srv.Empty, dummy=True)
  clone_srv('show_windows'     , std_srvs.srv.Empty, dummy=True)
  clone_srv('hide_windows'     , std_srvs.srv.Empty, dummy=True)
  clone_srv('start_record'     , std_srvs.srv.Empty, dummy=True)
  clone_srv('stop_record'      , std_srvs.srv.Empty, dummy=True)
  clone_srv('set_video_prefix' , fingervision_msgs.srv.SetString, dummy=True)
  clone_srv('set_frame_skip'   , fingervision_msgs.srv.SetInt32, dummy=True)
  clone_srv('take_snapshot'    , fingervision_msgs.srv.TakeSnapshot, dummy=True)
  clone_srv('stop_detect_obj'  , std_srvs.srv.Empty, dummy=True)
  clone_srv('start_detect_obj' , std_srvs.srv.Empty, dummy=True)
  clone_srv('clear_obj'        , std_srvs.srv.Empty, dummy=True)
  clone_srv('req_calibrate'     , fingervision_msgs.srv.SetStringInt32, dummy=True)
  clone_srv('req_initialize'    , fingervision_msgs.srv.SetStringInt32, dummy=True)
  clone_srv('set_dim_level'     , fingervision_msgs.srv.SetStringInt32, dummy=True)
  clone_srv('set_trackbar_mode' , fingervision_msgs.srv.SetStringInt32, dummy=True)
  clone_srv('save_parameters'   , fingervision_msgs.srv.SetString, dummy=True)
  clone_srv('load_parameters'   , fingervision_msgs.srv.SetString, dummy=True)
  clone_srv('save_calibration'  , fingervision_msgs.srv.SetStringInt32, dummy=True)
  clone_srv('load_calibration'  , fingervision_msgs.srv.SetStringInt32, dummy=True)

  rospy.spin()
