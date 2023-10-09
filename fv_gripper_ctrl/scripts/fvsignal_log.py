#!/usr/bin/python
#\file    fvsignal_log.py
#\brief   Logger tool of fvsignals.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.27, 2023
import os
import copy
import sys
import threading
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
import rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('fv_gripper_ctrl'),'scripts'))
from fv_gripper_ctrl import DecodeNamedVariableMsg,DecodeNamedVariableListMsg
from ay_py.core import LoadYAML, TimeStr
import fingervision_msgs.msg
import std_msgs.msg
import numpy as np


class TFVSignalListener(object):
  def __init__(self, fvsignal_list, data_skip):
    self.fvsignal_list= fvsignal_list
    self.data_skip= data_skip
    self.signal_names= [signal_name for (signal_name,label,axis,index) in self.fvsignal_list]

    for (topic,msg_type) in (
        ('gripper_pos',std_msgs.msg.Float64),
        ('target_pos',std_msgs.msg.Float64) ):
      setattr(self, topic, None)
      sub= rospy.Subscriber('/fv_gripper_ctrl/{}'.format(topic), msg_type, lambda msg,topic=topic:self.Callback(topic,msg))
      setattr(self, 'sub_{}'.format(topic), sub)

    self.fvsignals= None
    self.fvsignals_header= None
    self.sub_fvsignals= rospy.Subscriber('/fv_gripper_ctrl/fvsignals', fingervision_msgs.msg.NamedVariableListStamped, self.CallbackFVSignals)

  def Callback(self, topic, msg):
    setattr(self, topic, msg.data)
    setattr(self, topic+'_header', getattr(msg,'header',None))

  def CallbackFVSignals(self, msg):
    if self.data_skip>0 and msg.header.seq%self.data_skip!=0:  return
    self.fvsignals= msg.data
    self.fvsignals_header= msg.header
    self.UpdateValues()

  def Decode(self, names):
    fvsignals,time_stamp= self.fvsignals.data,self.fvsignals_header.stamp.to_sec()
    if fvsignals is None:  return None
    data= [DecodeNamedVariableMsg(d) for d in fvsignals if d.name in names]
    decoded= {name:value for (name,value) in data}
    decoded['gripper_pos']= self.gripper_pos
    decoded['target_pos']= self.target_pos
    return decoded, time_stamp

  @staticmethod
  def ToValue(fvsignals_decoded, signal_name, index):
    if fvsignals_decoded[signal_name] is None:  return None
    return (fvsignals_decoded[signal_name] if index is None
            else fvsignals_decoded[signal_name][index] if isinstance(index,int)
            else fvsignals_decoded[signal_name][tuple(index)] )

  def UpdateValues(self):
    pass

class TFVSignalListenerForLog(TFVSignalListener):
  def __init__(self, file_name, fvsignal_list, data_skip, with_label_line=True):
    self.file_name= file_name
    self.with_label_line= with_label_line
    super(TFVSignalListenerForLog,self).__init__(fvsignal_list, data_skip)

  def __enter__(self, *args, **kwargs):
    self.fp= open(self.file_name,'w')
    if self.with_label_line:
      labels= [label for (signal_name,label,axis,index) in self.fvsignal_list]
      self.fp.write('%time {}\n'.format(' '.join(labels)))
    return self

  def __exit__(self, *args, **kwargs):
    self.fp.close()

  def UpdateValues(self):
    fvsignals_decoded,time_stamp= self.Decode(self.signal_names)
    if fvsignals_decoded is None:  return False
    #print fvsignals_decoded

    new_values= [self.ToValue(fvsignals_decoded, signal_name, index)
                  for (signal_name,label,axis,index) in self.fvsignal_list]
    self.fp.write('{} {}\n'.format(time_stamp,' '.join(map(str,new_values))))


if __name__=='__main__':
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  file_prefix= get_arg('-file_prefix=',get_arg('--file_prefix=','/tmp/log-'))
  no_label_line= '-no_label_line' in sys.argv or '--no_label_line' in sys.argv
  data_skip= int(get_arg('-data_skip=',get_arg('--data_skip=',0)))
  logs= get_arg('-logs=',get_arg('--logs=',None))
  assert(file_prefix is not None)
  if logs is not None:
    if os.path.exists(logs):
      logs= LoadYAML(logs)
    else:
      logs= eval(logs)
  if logs is None:
    #List of (signal name, label, axis (1 or 2), tuple of value-index (specify None for scalar), enabled).
    #NOTE: axis is only used by plotter, ignored by the logger.
    logs= [('fv.slip','slip',1,None, True),
            ('fv.area','area',1,None, True),
            ('fv.center_l','center_l_y',1,1, True),
            ('gripper_pos','gpos',2,None, True),
            ('target_pos','gpos_trg',2,None, True)]
  print 'logs=',logs

  logs= [(signal_name,label,axis,index) for (signal_name,label,axis,index,enabled) in logs if enabled]

  rospy.init_node('fvsignal_log')
  file_name= '{}{}.dat'.format(file_prefix, TimeStr('short2'))
  with TFVSignalListenerForLog(file_name, logs, data_skip, not no_label_line) as fvsignal_listener:
    rospy.spin()

