#!/usr/bin/python3
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
from ay_py.core import LoadYAML, TimeStr, CPrint, PrintException
from ay_py.ros import TROSUtil
import std_msgs.msg
import std_srvs.srv
import fingervision_msgs.msg
import fingervision_msgs.srv
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
      sub= rospy.Subscriber('/fv_gripper_ctrl/{}'.format(topic), msg_type, lambda msg,topic=topic:self.Callback(topic,msg), queue_size=1, tcp_nodelay=True)
      setattr(self, 'sub_{}'.format(topic), sub)

    self.fvsignals= None
    self.fvsignals_header= None
    self.sub_fvsignals= rospy.Subscriber('/fv_gripper_ctrl/fvsignals', fingervision_msgs.msg.NamedVariableListStamped, self.CallbackFVSignals, queue_size=1, tcp_nodelay=True)

  def Callback(self, topic, msg):
    setattr(self, topic, msg.data)
    setattr(self, topic+'_header', getattr(msg,'header',None))

  def CallbackFVSignals(self, msg):
    if self.data_skip>0 and msg.header.seq%self.data_skip!=0:  return
    self.fvsignals= msg.data
    self.fvsignals_header= msg.header
    self.UpdateValues()

  def Decode(self, names):
    fvsignals,fv_time_stamp= self.fvsignals.data,self.fvsignals_header.stamp.to_sec()
    if fvsignals is None:  return None, None, None
    data= [DecodeNamedVariableMsg(d) for d in fvsignals if d.name in names]
    decoded= {name:value for (name,value) in data}
    decoded['gripper_pos']= self.gripper_pos
    decoded['target_pos']= self.target_pos
    time_stamp= rospy.Time.now().to_sec()
    return decoded, time_stamp, fv_time_stamp

  @staticmethod
  def ToValue(fvsignals_decoded, signal_name, index):
    if signal_name not in fvsignals_decoded or fvsignals_decoded[signal_name] is None:  return None
    try:
      return (fvsignals_decoded[signal_name] if index is None
              else fvsignals_decoded[signal_name][index] if isinstance(index,int)
              else fvsignals_decoded[signal_name][tuple(index)] )
    except IndexError:
      return None

  def UpdateValues(self):
    pass

class TFVSignalListenerForLog(TFVSignalListener):
  def __init__(self, file_name, fvsignal_list, data_skip, with_label_line=True):
    self.file_name= file_name
    self.with_label_line= with_label_line
    super(TFVSignalListenerForLog,self).__init__(fvsignal_list, data_skip)
    self.logging= True
    self.fp= None
    self.locker_fp= threading.RLock()

  def __enter__(self, *args, **kwargs):
    dir_name= os.path.dirname(self.file_name)
    if dir_name and not os.path.exists(dir_name):
      os.makedirs(dir_name)
    with self.locker_fp:
      self.fp= open(self.file_name,'w')
      if self.with_label_line:
        labels= [label for (signal_name,label,axis,index) in self.fvsignal_list]
        self.fp.write('%time fv_time {}\n'.format(' '.join(labels)))
    print(f'Start logging to {self.file_name}')
    return self

  def __exit__(self, *args, **kwargs):
    with self.locker_fp:
      self.fp.close()
      self.fp= None
    print(f'Finished logging to {self.file_name}')

  def UpdateValues(self):
    if not self.logging:  return False

    fvsignals_decoded, time_stamp, fv_time_stamp= self.Decode(self.signal_names)
    if fvsignals_decoded is None:  return False
    #print fvsignals_decoded

    new_values= [self.ToValue(fvsignals_decoded, signal_name, index)
                  for (signal_name,label,axis,index) in self.fvsignal_list]
    with self.locker_fp:
      if self.fp is None:
        return False
      self.fp.write('{} {} {}\n'.format(time_stamp, fv_time_stamp,' '.join(map(str,new_values))))


#Make a log file name from the prefix.
def MakeLogFileName(file_prefix, time_stamp_fmt='short2'):
  return '{}{}.dat'.format(file_prefix, TimeStr(time_stamp_fmt))

#Load (convert) signal_list from a text(str).
def LoadSignalListFromText(signal_list):
  signal_list= eval(signal_list)
  signal_list= [(signal_name,label,axis,index) for (signal_name,label,axis,index,enabled) in signal_list if enabled]
  return signal_list

#Load signal_list from a file.
def ReloadSignalList(signal_list_file):
  signal_list= LoadYAML(signal_list_file)
  signal_list= [(signal_name,label,axis,index) for (signal_name,label,axis,index,enabled) in signal_list if enabled]
  return signal_list


'''
Interface ROS node of FV signal logger.
This node is designed to work in background and make logs only when requested.
The subscription to the data (such as fvsignals) is done only during logging
to minimize the CPU usage.
'''
class TFVSignalLoggerNode(TROSUtil):
  def __init__(self, log_file_prefix, signal_list_file, signal_list, data_skip, with_label_line=True):
    super(TFVSignalLoggerNode,self).__init__()
    self.log_file_prefix  = log_file_prefix
    self.signal_list_file = signal_list_file
    self.signal_list      = signal_list
    self.data_skip        = data_skip
    self.with_label_line  = with_label_line
    self.fvsignal_listener= None

  def __del__(self):
    self.Cleanup()
    if TFVSignalLoggerNode is not None:  super(TFVSignalLoggerNode,self).__del__()
    print('TFVSignalLoggerNode: done',self)

  def Cleanup(self):
    if TFVSignalLoggerNode is not None:  super(TFVSignalLoggerNode,self).Cleanup()

  def Setup(self):
    self.AddPub('status','~status',std_msgs.msg.Empty,queue_size=1)

    def add_srv_s(name, f_method):
      self.AddSrv(name, f'~{name}', fingervision_msgs.srv.SetString,
                  lambda req:(f_method(req.data),
                              fingervision_msgs.srv.SetStringResponse())[-1])
    def add_srv_e(name, f_method):
      self.AddSrv(name, f'~{name}', std_srvs.srv.Empty,
                  lambda req:(f_method(), std_srvs.srv.EmptyResponse())[-1])
    add_srv_s('set_log_file_prefix', self.SetLogFilePrefix  )
    add_srv_s('set_signal_list_file', self.SetSignalListFile )
    add_srv_s('set_signal_list', self.SetSignalList )
    add_srv_e('reload_signal_list', self.ReloadSignalList )
    add_srv_e('start', self.Start )
    add_srv_e('pause', self.Pause )
    add_srv_e('finish', self.Finish )

    self.fvsignal_listener= None

  def SetLogFilePrefix(self, log_file_prefix):
    self.log_file_prefix= log_file_prefix

  def SetSignalListFile(self, signal_list_file):
    self.signal_list_file= signal_list_file

  def SetSignalList(self, signal_list):
    try:
      self.signal_list= LoadSignalListFromText(signal_list)
    except Exception as e:
      PrintException(e)

  def ReloadSignalList(self):
    if self.signal_list_file is None:
      CPrint(4, f'ReloadSignalList is requested, but signal_list_file is not specified.')
      return
    if not os.path.exists(self.signal_list_file):
      CPrint(4, f'In ReloadSignalList, signal_list_file does not exist: {self.signal_list_file}.')
      return
    self.signal_list= ReloadSignalList(signal_list_file)

  def Start(self):
    if self.fvsignal_listener is not None:
      self.fvsignal_listener.logging= True
      return
    file_name= MakeLogFileName(self.log_file_prefix)
    self.fvsignal_listener= TFVSignalListenerForLog(file_name, self.signal_list, self.data_skip, self.with_label_line)
    self.fvsignal_listener.__enter__()

  def Pause(self):
    if self.fvsignal_listener is not None:
      self.fvsignal_listener.logging= False

  def Finish(self):
    if self.fvsignal_listener is not None:
      self.fvsignal_listener.logging= False
      self.fvsignal_listener.__exit__()
      self.fvsignal_listener= None

  #Waiting loop where the status topic is published at a low frequency.
  def Spin(self):
    rate_adjuster= rospy.Rate(1)
    while not rospy.is_shutdown():
      self.pub.status.publish(std_msgs.msg.Empty())
      rate_adjuster.sleep()


if __name__=='__main__':
  def get_arg(opt_name, default):
    exists= [a.startswith(opt_name) for a in sys.argv]
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  #File name prefix of a log file.
  file_prefix= get_arg('-file_prefix=',get_arg('--file_prefix=','/tmp/log-'))
  #Disable to put a label line at the beginning of the data.
  no_label_line= '-no_label_line' in sys.argv or '--no_label_line' in sys.argv
  #Interval (int) to skip the data sequence to reduce the computation cost.
  data_skip= int(get_arg('-data_skip=',get_arg('--data_skip=',0)))
  #List of signal names to log.
  signal_list= get_arg('-logs=',get_arg('--logs=',None))
  #Making the node executed persistently (which can accept multiple log requests via ROS services).
  persistent_mode= '-persistent_mode' in sys.argv or '--persistent_mode' in sys.argv
  assert(file_prefix is not None)
  signal_list_file= None
  if signal_list is not None:
    if os.path.exists(signal_list):
      signal_list_file= signal_list
      signal_list= ReloadSignalList(signal_list_file)
    else:
      signal_list= LoadSignalListFromText(signal_list)
  if signal_list is None:
    #List of (signal name, label, axis (1 or 2), tuple of value-index (specify None for scalar), enabled).
    #NOTE: axis is only used by plotter, ignored by the logger.
    signal_list= [('fv.slip','slip',1,None, True),
                  ('fv.area','area',1,None, True),
                  ('fv.center_l','center_l_y',1,1, True),
                  ('gripper_pos','gpos',2,None, True),
                  ('target_pos','gpos_trg',2,None, True)]
    signal_list= [(signal_name,label,axis,index) for (signal_name,label,axis,index,enabled) in signal_list if enabled]
  print(f'Logger: signal_list= {signal_list}')

  rospy.init_node('fvsignal_log')
  print(f'Logger: persistent_mode= {persistent_mode}')

  if not persistent_mode:
    file_name= MakeLogFileName(file_prefix)
    with TFVSignalListenerForLog(file_name, signal_list, data_skip, not no_label_line) as fvsignal_listener:
      rospy.spin()
  else:
    logger_node= TFVSignalLoggerNode(file_prefix, signal_list_file, signal_list, data_skip, not no_label_line)
    try:
      logger_node.Setup()
      #rospy.spin()
      logger_node.Spin()
    finally:
      logger_node.Finish()



