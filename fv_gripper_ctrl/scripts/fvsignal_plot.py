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
import fingervision_msgs.srv
import numpy as np
import matplotlib.pyplot as plt


class TFVSignalListener(object):
  def __init__(self):
    self.sub= rospy.Subscriber('/fv_gripper_ctrl/fvsignals', fingervision_msgs.msg.NamedVariableList, self.Callback)
    self.fvsignals= None

  def Callback(self, msg):
    self.fvsignals= msg.data

  def Decode(self, names):
    fvsignals= self.fvsignals
    if fvsignals is None:  return None
    data= [DecodeNamedVariableMsg(d) for d in fvsignals if d.name in names]
    return {name:value for (name,value) in data}


if __name__=='__main__':
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  #port= int(get_arg('-port=',get_arg('--port=',5020)))
  #no_kbhit= True if '-no_kbhit' in sys.argv or '--no_kbhit' in sys.argv else False
  title= ''
  xlabel= 'time[s]'
  ylabel= 'y1'
  y2label= 'y2'
  plots= [('fv.slip','slip',None), ('fv.area','area',None), ('fv.center','center_y',1)]

  fig= plt.figure(figsize=(7,5))
  ax1= fig.add_subplot(1,1,1)

  signal_names= [signal_name for (signal_name,plot_label,index) in plots]

  rospy.init_node('fvsignal_plot')
  fvsignal_listener= TFVSignalListener()
  #TODO:FIXME:Add gpos, gpos_trg

  plot_values= {}
  rate_adjuster= rospy.Rate(20)
  while not rospy.is_shutdown():
    fvsignals_decoded= fvsignal_listener.Decode(signal_names)
    if fvsignals_decoded is None:  continue
    print fvsignals_decoded

    new_values= {plot_label: fvsignals_decoded[signal_name] if index is None
                              else fvsignals_decoded[signal_name][index] if isinstance(index,int)
                              else fvsignals_decoded[signal_name][tuple(index)]
                  for (signal_name,plot_label,index) in plots
                    if fvsignals_decoded[signal_name] is not None}
    new_time= rospy.Time.now().to_sec()
    print new_time,new_values

    for plot_label,value in new_values.iteritems():
      if plot_label in plot_values:
        plot_values[plot_label][0].append(new_time)
        plot_values[plot_label][1].append(value)
      else:
        plot_values[plot_label]= [[new_time],[value]]

    #print plot_values
    ax1.cla()
    for plot_label,[times,values] in plot_values.iteritems():
      ax1.plot(times,values, linewidth=2, label=plot_label)

    ax1.set_title(title)
    ax1.set_xlabel(xlabel)
    ax1.set_ylabel(ylabel)
    #ax1.set_ylim(bottom=-1.2,top=1.2)
    ax1.legend()
    plt.pause(0.05)
    #rate_adjuster.sleep()
