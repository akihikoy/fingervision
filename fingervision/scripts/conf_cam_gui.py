#!/usr/bin/python
#\file    conf_cam_gui.py
#\brief   Simple GUI tool for configuring cameras.
#         This tool does not open a camera, so you can use this together with
#         other camera apps such as fingervision programs.
#         This tool uses uvcdynctrl for camera control, and ay_py ROS package for GUI.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.29, 2021
import roslib
roslib.load_manifest('ay_py')
from ay_py.core import CPrint
from ay_py.tool.py_panel import TSimplePanel, InitPanelApp, RunPanelApp, AskYesNoDialog, QtCore, QtGui
import sys
import subprocess
import re

def ExecCmd(cmd):
  p= subprocess.Popen(cmd, stdout=subprocess.PIPE)
  (stdout, stderr)= p.communicate()
  exit_code= p.wait()
  return stdout.strip() if stdout is not None else stdout, stderr.strip() if stderr is not None else stderr, exit_code
'''
  Exposure, Auto
    ID      : 0x0000000f,
    Type    : Choice,
    Flags   : { CAN_READ, CAN_WRITE },
    Values  : { 'Auto Mode'[0], 'Manual Mode'[1], 'Shutter Priority Mode'[2], 'Aperture Priority Mode'[3] },
    Default : 3
  Exposure (Absolute)
    ID      : 0x00000011,
    Type    : Dword,
    Flags   : { CAN_READ, CAN_WRITE },
    Values  : [ 1 .. 1331, step size: 1 ],
    Default : 120
'''

def GetControls(cam_dev):
  stdout,stderr,ec= ExecCmd(['uvcdynctrl', '-d', cam_dev, '-c'])
  ctrls= []
  for s in stdout.split('\n'):
    if 'Listing available controls' in s:  continue
    ctrls.append(s.strip())
  ctrl_details= {ctrl:{} for ctrl in ctrls}
  stdout,stderr,ec= ExecCmd(['uvcdynctrl', '-d', cam_dev, '-c', '-v'])
  curr_ctrl= None
  for s in stdout.split('\n'):
    if 'Listing available controls' in s:  continue
    s= s.strip()
    if s in ctrls:
      curr_ctrl= s
    elif curr_ctrl is not None:
      idx_colon= s.find(':')
      param_name= s[:idx_colon-1].strip()
      value= s[idx_colon+1:].strip()
      if value[-1]==',':  value= value[:-1]
      ctrl_details[curr_ctrl][param_name]= value
  for ctrl,details in ctrl_details.iteritems():
    details['Flags']= re.findall(r'[a-zA-Z0-9_]+', details['Flags'])
    details['Default']= int(details['Default'])
    if details['Type']=='Choice':
      values= re.findall(r'\'[a-zA-Z0-9\s]+\'\[[0-9]+\]', details['Values'])
      matches= [re.match(r'\'([a-zA-Z0-9\s]+)\'\[([0-9]+)\]', value) for value in values]
      values= {int(match.group(2)): match.group(1) for match in matches}
      details['Values']= [values[i] if i in values else None for i in range(sorted(values.keys())[-1])]
    elif details['Type'] in ('Boolean', 'Dword'):
      #(min,max,step) tuple.
      details['Values']= map(int, re.match(r'\[\s*([\-\+0-9]+)\s*\.\.\s*([\-\+0-9]+)\s*,\s*step size\:\s*([\-\+0-9]+)\s*\]', details['Values']).groups())
  return ctrls, ctrl_details

def GetCtrlValue(cam_dev, ctrl):
  stdout,stderr,ec= ExecCmd(['uvcdynctrl', '-d', cam_dev, '-g', ctrl])
  return int(stdout.strip())

def SetCtrlValue(cam_dev, ctrl, value):
  #NOTE: '--' is neccessary for nevative values (not neccessary for positive ones).
  stdout,stderr,ec= ExecCmd(['uvcdynctrl', '-d', cam_dev, '-s', ctrl, '--', str(value)])


def GenerateImgCtrlWidges(cam_dev):
  ctrls,ctrl_details= GetControls(cam_dev)
  print 'ctrls:',ctrls
  print 'ctrl_details:',ctrl_details

  ctrl_values= {ctrl: GetCtrlValue(cam_dev, ctrl) for ctrl in ctrls}
  for ctrl in ctrls:
    value= ctrl_values[ctrl]
    print ctrl,'=',value,ctrl_details[ctrl]['Values'][value] if ctrl_details[ctrl]['Type']=='Choice' else ''

  def onvaluechange(ctrl, value):
    print 'Setting',ctrl,value
    SetCtrlValue(cam_dev, ctrl, value)

  widges_img_ctrl= {}
  layout_img_ctrl= []
  for i_ctrl,ctrl in enumerate(ctrls):
    widges_img_ctrl['label_'+ctrl]= (
      'label',{
        'text': ctrl+':',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')})
    layout_img_ctrl.append(('label_'+ctrl,i_ctrl,0,'right'))
    if ctrl_details[ctrl]['Type']=='Choice':
      widges_img_ctrl['combobox_'+ctrl]= (
        'combobox',{
          'options': ctrl_details[ctrl]['Values'],
          'index': ctrl_values[ctrl],
          #'font_size_range': (8,24),
          'size_adjust_policy': 'all_contents',
          'onactivated': lambda w,obj,ctrl=ctrl:onvaluechange(ctrl,obj.currentIndex()) })
      #layout_img_ctrl.append(('boxh',None, ('label_'+ctrl,'combobox_'+ctrl)))
      layout_img_ctrl.append(('combobox_'+ctrl,i_ctrl,1))
    elif ctrl_details[ctrl]['Type']=='Boolean':
      widges_img_ctrl['checkbox_'+ctrl]= (
        'checkbox',{
          'text': '',
          'checked': ctrl_values[ctrl],
          'onclick': lambda w,obj,ctrl=ctrl:onvaluechange(ctrl,1 if obj.isChecked() else 0) })
      #layout_img_ctrl.append(('boxh',None, ('label_'+ctrl,'checkbox_'+ctrl)))
      layout_img_ctrl.append(('checkbox_'+ctrl,i_ctrl,1))
    elif ctrl_details[ctrl]['Type']=='Dword':
      widges_img_ctrl['slider_'+ctrl]= (
        'sliderh',{
          'range': ctrl_details[ctrl]['Values'],
          'value': ctrl_values[ctrl],
          'n_labels': 3,
          'slider_style': 1,
          'onvaluechange': lambda w,obj,ctrl=ctrl:onvaluechange(ctrl,obj.value()) })
      #layout_img_ctrl.append(('boxh',None, ('label_'+ctrl,'slider_'+ctrl)))
      layout_img_ctrl.append(('slider_'+ctrl,i_ctrl,1))
  layout_img_ctrl= ('grid',None, layout_img_ctrl)

  return widges_img_ctrl,layout_img_ctrl,ctrls,ctrl_details

if __name__=='__main__':
  cam_dev= sys.argv[1] if len(sys.argv)>1 else '/dev/video0'

  widges_img_ctrl,layout_img_ctrl,ctrls,ctrl_details= GenerateImgCtrlWidges(cam_dev)

  widgets_common= {
    'label_device': (
      'label',{
        'text': 'Device: '+cam_dev,
        'size_policy': ('expanding', 'minimum')}),
    'btn_exit': (
      'button',{
        'text': 'Exit',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: w.close(), }),
    }

  layout_main= (
    'boxv',None,(
      ('boxh',None, ('label_device','btn_exit')),
      layout_img_ctrl,
      ))

  app= InitPanelApp()
  win_size= (400,400)
  panel= TSimplePanel('Camera Control', size=win_size, font_height_scale=800.0)
  panel.AddWidgets(widgets_common)
  panel.AddWidgets(widges_img_ctrl)
  panel.Construct(layout_main)
  RunPanelApp()


