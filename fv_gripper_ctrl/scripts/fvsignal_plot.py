#!/usr/bin/python
#\file    fvsignal_plot.py
#\brief   Realtime plot tool of fvsignals.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.08, 2023
#Example command line:
#  ./fvsignal_plot.py --plots="[('fv.slip','slip',1,None),('gripper_pos','gpos',2,None),('target_pos','gpos_trg',2,None)]"
import os
import copy
import sys
import threading
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
import rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('fv_gripper_ctrl'),'scripts'))
from fv_gripper_ctrl import DecodeNamedVariableMsg,DecodeNamedVariableListMsg
from ay_py.core import LoadYAML
import fingervision_msgs.msg
import std_msgs.msg
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as plt_cols


class TFVSignalListener(object):
  def __init__(self, plot_config, data_skip):
    self.plot_value_locker= threading.RLock()

    self.plot_config= plot_config
    self.data_skip= data_skip
    self.plot_values= {}
    self.signal_names= [signal_name for (signal_name,plot_label,axis,index) in plot_config]

    for (topic,msg_type) in (
        ('gripper_pos',std_msgs.msg.Float64),
        ('target_pos',std_msgs.msg.Float64) ):
      setattr(self, topic, None)
      cb= self.UpdatePlotValues if topic=='fvsignals' else None
      sub= rospy.Subscriber('/fv_gripper_ctrl/{}'.format(topic), msg_type, lambda msg,topic=topic,cb=cb:self.Callback(topic,msg,cb))
      setattr(self, 'sub_{}'.format(topic), sub)

    self.fvsignals= None
    self.fvsignals_header= None
    self.sub_fvsignals= rospy.Subscriber('/fv_gripper_ctrl/fvsignals', fingervision_msgs.msg.NamedVariableListStamped, self.CallbackFVSignals)

  def Callback(self, topic, msg, cb):
    setattr(self, topic, msg.data)
    setattr(self, topic+'_header', getattr(msg,'header',None))
    if cb is not None:  cb()

  def CallbackFVSignals(self, msg):
    if self.data_skip>0 and msg.header.seq%self.data_skip!=0:  return
    self.fvsignals= msg.data
    self.fvsignals_header= msg.header
    self.UpdatePlotValues()

  def Decode(self, names):
    fvsignals,time_stamp= self.fvsignals.data,self.fvsignals_header.stamp.to_sec()
    if fvsignals is None:  return None
    data= [DecodeNamedVariableMsg(d) for d in fvsignals if d.name in names]
    decoded= {name:value for (name,value) in data}
    decoded['gripper_pos']= self.gripper_pos
    decoded['target_pos']= self.target_pos
    return decoded, time_stamp

  def UpdatePlotValues(self):
    fvsignals_decoded,time_stamp= self.Decode(self.signal_names)
    if fvsignals_decoded is None:  return False
    #print fvsignals_decoded

    new_values= {plot_label: fvsignals_decoded[signal_name] if index is None
                              else fvsignals_decoded[signal_name][index] if isinstance(index,int)
                              else fvsignals_decoded[signal_name][tuple(index)]
                  for (signal_name,plot_label,axis,index) in self.plot_config
                    if fvsignals_decoded[signal_name] is not None}

    plot_values= copy.deepcopy(self.plot_values)
    for plot_label,value in new_values.iteritems():
      if plot_label in plot_values:
        plot_values[plot_label][0].append(time_stamp)
        plot_values[plot_label][1].append(value)
        while len(plot_values[plot_label][0])>plot_points:
          plot_values[plot_label][0].pop(0)
          plot_values[plot_label][1].pop(0)
      else:
        plot_values[plot_label]= [[time_stamp],[value]]
    with self.plot_value_locker:
      self.plot_values= plot_values
    return True

  def PlotValues(self):
    with self.plot_value_locker:
      return self.plot_values

if __name__=='__main__':
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  #no_kbhit= True if '-no_kbhit' in sys.argv or '--no_kbhit' in sys.argv else False
  plot_rate= float(get_arg('-rate=',get_arg('--rate=',20.0)))
  plot_points= int(get_arg('-points=',get_arg('--points=',500)))
  data_skip= int(get_arg('-data_skip=',get_arg('--data_skip=',5)))
  title= get_arg('-title=',get_arg('--title=',''))
  xlabel= get_arg('-xlabel=',get_arg('--xlabel=',None))
  ylabel= get_arg('-xlabel=',get_arg('--xlabel=',None))
  y2label= get_arg('-xlabel=',get_arg('--xlabel=',None))
  plots= get_arg('-plots=',get_arg('--plots=',None))
  if plots is not None:
    if os.path.exists(plots):
      plots= LoadYAML(plots)
    else:
      plots= eval(plots)
  if plots is None:
    #List of (signal name, label, axis (1 or 2), tuple of value-index (specify None for scalar), enabled).
    plots= [('fv.slip','slip',1,None, True),
            ('fv.area','area',1,None, True),
            ('fv.center','center_y',1,1, True),
            ('gripper_pos','gpos',2,None, True),
            ('target_pos','gpos_trg',2,None, True)]
  print 'plots=',plots

  plots= [(signal_name,plot_label,axis,index) for (signal_name,plot_label,axis,index,enabled) in plots if enabled]

  rospy.init_node('fvsignal_plot')
  fvsignal_listener= TFVSignalListener(plots, data_skip)

  #t_start= rospy.Time.now().to_sec()

  fig= plt.figure(figsize=(7,5))
  ax1= fig.add_subplot(1,1,1)
  ax2= ax1.twinx()

  if xlabel is None:
    #xlabel= 'time [s] (+ {})'.format(t_start)
    xlabel= 'time [s]'
  if ylabel is None:
    ylabel= ','.join([plot_label for (signal_name,plot_label,axis,index) in plots if axis==1])
  if y2label is None:
    y2label= ','.join([plot_label for (signal_name,plot_label,axis,index) in plots if axis==2])

  rate_adjuster= rospy.Rate(plot_rate)
  while not rospy.is_shutdown():
    plot_values= fvsignal_listener.PlotValues()

    #print plot_values
    ax1.cla()
    ax2.cla()

    lines= []
    col_scheme= plt_cols.TABLEAU_COLORS
    for i,(signal_name,plot_label,axis,index) in enumerate(fvsignal_listener.plot_config):
      times,values= plot_values[plot_label]
      col= col_scheme.values()[i%len(col_scheme)]
      ax= (None,ax1,ax2)[axis]
      lines+= ax.plot(times,values, color=col, linewidth=2, label=plot_label)

    #ax1.legend(loc='upper left', bbox_to_anchor=(0.0,1.0))
    #ax2.legend(loc='upper left', bbox_to_anchor=(0.0,0.8))
    ax2.legend(lines, [l.get_label() for l in lines], loc='upper left')

    ax1.set_title(title)
    ax1.set_xlabel(xlabel)
    ax1.set_ylabel(ylabel)
    #ax1.set_ylim(bottom=-1.2,top=1.2)
    ax2.set_ylabel(y2label)
    fig.tight_layout()
    plt.pause(0.001)
    rate_adjuster.sleep()
