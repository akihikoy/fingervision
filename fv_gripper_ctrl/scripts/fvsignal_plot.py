#!/usr/bin/python
#\file    fvsignal_plot.py
#\brief   Realtime plot tool of fvsignals.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.08, 2023
import os
import sys
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
import rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('fv_gripper_ctrl'),'scripts'))
from fv_gripper_ctrl import DecodeNamedVariableMsg,DecodeNamedVariableListMsg
import fingervision_msgs.msg
import std_msgs.msg
import numpy as np
import matplotlib.pyplot as plt


class TFVSignalListener(object):
  def __init__(self):
    for (topic,msg_type) in (
        ('gripper_pos',std_msgs.msg.Float64),
        ('target_pos',std_msgs.msg.Float64),
        ('fvsignals',fingervision_msgs.msg.NamedVariableList) ):
      setattr(self, topic, None)
      sub= rospy.Subscriber('/fv_gripper_ctrl/{}'.format(topic), msg_type, lambda msg,topic=topic:self.Callback(topic,msg))
      setattr(self, 'sub_{}'.format(topic), sub)

  def Callback(self, topic, msg):
    setattr(self, topic, msg.data)

  def Decode(self, names):
    fvsignals= self.fvsignals
    if fvsignals is None:  return None
    data= [DecodeNamedVariableMsg(d) for d in fvsignals if d.name in names]
    decoded= {name:value for (name,value) in data}
    decoded['gripper_pos']= self.gripper_pos
    decoded['target_pos']= self.target_pos
    return decoded


if __name__=='__main__':
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  #no_kbhit= True if '-no_kbhit' in sys.argv or '--no_kbhit' in sys.argv else False
  plot_rate= float(get_arg('-rate=',get_arg('--rate=',20.0)))
  plot_points= int(get_arg('-points=',get_arg('--points=',200)))
  title= get_arg('-title=',get_arg('--title=',''))
  xlabel= get_arg('-xlabel=',get_arg('--xlabel=',None))
  ylabel= get_arg('-xlabel=',get_arg('--xlabel=',None))
  y2label= get_arg('-xlabel=',get_arg('--xlabel=',None))
  plots= get_arg('-plots=',get_arg('--plots=',''))
  plots= None if plots=='' else [p.split(':') for p in plots.split(',')]
  if plots is None:
    #List of signal name, label, axis, tuple of value-index (specify None for scalar).
    plots= [('fv.slip','slip',1,None),
            ('fv.area','area',1,None),
            ('fv.center','center_y',1,1),
            ('gripper_pos','gpos',2,None),
            ('target_pos','gpos_trg',2,None)]

  fig= plt.figure(figsize=(7,5))
  ax1= fig.add_subplot(1,1,1)
  ax2= ax1.twinx()

  rospy.init_node('fvsignal_plot')
  fvsignal_listener= TFVSignalListener()

  signal_names= [signal_name for (signal_name,plot_label,axis,index) in plots]
  axis_map= {plot_label:axis for (signal_name,plot_label,axis,index) in plots}
  t_start= rospy.Time.now().to_sec()
  if xlabel is None:
    xlabel= 'time [s] (+ {})'.format(t_start)
  if ylabel is None:
    ylabel= ','.join([plot_label for (signal_name,plot_label,axis,index) in plots if axis==1])
  if y2label is None:
    y2label= ','.join([plot_label for (signal_name,plot_label,axis,index) in plots if axis==2])

  plot_values= [{},{}]
  rate_adjuster= rospy.Rate(plot_rate)
  while not rospy.is_shutdown():
    fvsignals_decoded= fvsignal_listener.Decode(signal_names)
    if fvsignals_decoded is None:  continue
    print fvsignals_decoded

    new_values= {plot_label: fvsignals_decoded[signal_name] if index is None
                              else fvsignals_decoded[signal_name][index] if isinstance(index,int)
                              else fvsignals_decoded[signal_name][tuple(index)]
                  for (signal_name,plot_label,axis,index) in plots
                    if fvsignals_decoded[signal_name] is not None}
    new_time= rospy.Time.now().to_sec()-t_start
    print new_time,new_values

    for plot_label,value in new_values.iteritems():
      axis= axis_map[plot_label]-1
      if plot_label in plot_values[axis]:
        plot_values[axis][plot_label][0].append(new_time)
        plot_values[axis][plot_label][1].append(value)
        while len(plot_values[axis][plot_label])>plot_points:
          plot_values[axis][plot_label][0].pop()
          plot_values[axis][plot_label][1].pop()
      else:
        plot_values[axis][plot_label]= [[new_time],[value]]

    #print plot_values
    ax1.cla()
    ax2.cla()

    for plot_label,[times,values] in plot_values[0].iteritems():
      ax1.plot(times,values, linewidth=2, label=plot_label)

    for plot_label,[times,values] in plot_values[1].iteritems():
      ax2.plot(times,values, linewidth=2, label=plot_label)

    ax1.legend(loc='upper right', bbox_to_anchor=(1.0,1.0))
    ax2.legend(loc='upper right', bbox_to_anchor=(1.0,0.8))

    ax1.set_title(title)
    ax1.set_xlabel(xlabel)
    ax1.set_ylabel(ylabel)
    #ax1.set_ylim(bottom=-1.2,top=1.2)
    ax2.set_ylabel(y2label)
    plt.pause(0.001)
    rate_adjuster.sleep()
