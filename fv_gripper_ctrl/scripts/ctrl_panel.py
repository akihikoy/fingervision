#!/usr/bin/python
#\file    ctrl_panel.py
#\brief   FV+Gripper control panel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.18, 2022
'''
Setup:
  Link to FV cameras (e.g. /dev/video0, /dev/video1) to:
    /media/video_fv1
    /media/video_fv2
  Make directory: ~/data/data_gen/
  Put FV config files fvp300x_l.yaml, fvp300x_r.yaml in: ~/data/config/
  where:
    fvp300x_l.yaml:DevID: /media/video_fv1
    fvp300x_r.yaml:DevID: /media/video_fv2
  e.g.
    $ sudo ln -s /dev/video0 /media/video_fv1
    $ sudo ln -s /dev/video2 /media/video_fv2
    $ mkdir -p ~/data/data_gen/
    $ mkdir -p ~/data/config/ && cd ~/data/config/
    $ ln -s `rospack find ay_fv_extra`/config/fvp_4_l.yaml fvp300x_l.yaml
    $ ln -s `rospack find ay_fv_extra`/config/fvp_4_r.yaml fvp300x_r.yaml
'''
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import os,sys
import rospy
import rospkg
from ay_py.core import InsertDict, LoadYAML, SaveYAML
from ay_py.tool.py_panel import TSimplePanel, InitPanelApp, RunPanelApp, AskYesNoDialog, QtCore, QtGui
sys.path.append(os.path.join(rospkg.RosPack().get_path('ay_util'),'scripts'))
from proc_manager import TSubProcManager
from joy_fv import TJoyEmulator

class TSubProcManagerJoy(QtCore.QObject, TSubProcManager, TJoyEmulator):
  def __init__(self, node_name='ctrl_panel'):
    QtCore.QObject.__init__(self)
    TSubProcManager.__init__(self)
    TJoyEmulator.__init__(self)
    self.node_name= node_name

  def InitNode(self):
    rospy.init_node(self.node_name)
    rospy.sleep(0.1)


def UpdateProcList(pm,combobox):
  combobox.clear()
  for name,proc in pm.procs.iteritems():
    combobox.addItem('{0}/{1}'.format(name,proc.pid))

if __name__=='__main__':
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  gripper_type= get_arg('-gripper_type=',get_arg('--gripper_type=','RHP12RNGripper'))
  joy_dev= get_arg('-joy_dev=',get_arg('--joy_dev=','js0'))
  dxl_dev= get_arg('-dxl_dev=',get_arg('--dxl_dev=','USB0'))
  fullscreen= True if '-fullscreen' in sys.argv or '--fullscreen' in sys.argv else False
  is_sim= True if '-sim' in sys.argv or '--sim' in sys.argv else False

  RVIZ_CONFIG= os.environ['HOME']+'/.rviz/default.rviz'
  #Parameters:
  config={
    'GripperType': gripper_type,
    'JoyUSB': joy_dev,
    'DxlUSB': dxl_dev,
    'FV_L_DEV': '/media/video_fv1',
    'FV_R_DEV': '/media/video_fv2',
    'FV_BASE_DIR': '{}/data'.format(os.environ['HOME']),
    'FV_L_CONFIG': 'config/fvp300x_l.yaml',
    'FV_R_CONFIG': 'config/fvp300x_r.yaml',
    'FV_CTRL_CONFIG': '{}/data/config/fv_ctrl.yaml'.format(os.environ['HOME']),
    'IS_SIM': is_sim,
    }

  #List of commands (name: [[command/args],'fg'/'bg']).
  cmds= {
    'roscore': ['roscore','bg'],
    'fix_usb': ['sudo /sbin/fix_usb_latency.sh tty{DxlUSB}','fg'],
    'gripper': ['roslaunch ay_util gripper_selector.launch gripper_type:={GripperType} dxldev:=/dev/tty{DxlUSB}','bg'],
    'joy': ['rosrun joy joy_node joy_node {JoyUSB}','bg'],
    'fvp': ['roslaunch fingervision fvp_general.launch pkg_dir:={FV_BASE_DIR} config1:={FV_L_CONFIG} config2:={FV_R_CONFIG}','bg'],
    'fvp_file': ['roslaunch ay_fv_extra fvp_file1.launch','bg'],
    'config_fv_l': ['rosrun fingervision conf_cam2.py {FV_L_DEV} file:CameraParams:0:{FV_BASE_DIR}/{FV_L_CONFIG}','fg'],
    'config_fv_r': ['rosrun fingervision conf_cam2.py {FV_R_DEV} file:CameraParams:0:{FV_BASE_DIR}/{FV_R_CONFIG}','fg'],
    'start_record_l': ['rosservice call /fingervision/fvp_1_l/start_record','fg'],
    'start_record_r': ['rosservice call /fingervision/fvp_1_r/start_record','fg'],
    'stop_record_l': ['rosservice call /fingervision/fvp_1_l/stop_record','fg'],
    'stop_record_r': ['rosservice call /fingervision/fvp_1_r/stop_record','fg'],
    'rviz': ['rosrun rviz rviz -d {0}'.format(RVIZ_CONFIG),'bg'],
    'fv_gripper_ctrl': ['rosrun fv_gripper_ctrl fv_gripper_ctrl.py _gripper_type:={GripperType} _is_sim:={IS_SIM}','bg'],
    }
  if is_sim:
    cmds['fvp']= cmds['fvp_file']
    for c in ('fix_usb','gripper','config_fv_l','config_fv_r'):
      cmds[c][1]= None
  for key in cmds.iterkeys():
    if isinstance(cmds[key][0],str):
      cmds[key][0]= cmds[key][0].format(**config).split(' ')
  print config

  pm= TSubProcManagerJoy()
  run_cmd= lambda name: pm.RunBGProcess(name,cmds[name][0]) if cmds[name][1]=='bg' else\
                        pm.RunFGProcess(cmds[name][0]) if cmds[name][1]=='fg' else\
                        None
  stop_cmd= lambda name: pm.TerminateBGProcess(name)


  set_joy= lambda kind,value=None,is_active=0: pm.SetJoy(kind,value,is_active)



  #UI for configuring FV control parameters:
  ctrl_config= {
      #Common control parameters:
      'min_gstep': 0.0005,
      #Parameters used in fv.grasp:
      'grasp_th': 3,          #Threshold to stop.
      'grasp_filter_len': 4,  #Temporal filter length.
      'grasp_dstate_th': 3,   #Threshold of discrete state.
      #Parameters used in fv.hold, fv.pickup2a, fv.pickup2b:
      'hold_sensitivity_slip': 0.08,  #Sensitivity of slip detection (smaller is more sensitive).
      'hold_sensitivity_oc': 0.2,  #Sensitivity of object-center-movement detection (smaller is more sensitive).
      'hold_sensitivity_oo': 0.5,  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
      'hold_sensitivity_oa': 0.4,  #Sensitivity of object-area-change detection (smaller is more sensitive).
      #Parameters used in fv.openif:
      'openif_sensitivity_slip': 0.6,  #Sensitivity of slip detection (smaller is more sensitive).
      'openif_sensitivity_oc': 0.4,  #Sensitivity of object-center-movement detection (smaller is more sensitive).
      'openif_sensitivity_oo': 4.0,  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
      'openif_sensitivity_oa': 0.6,  #Sensitivity of object-area-change detection (smaller is more sensitive).
      'openif_sensitivity_force':0.9,  #Sensitivity of each force element; if the norm of force change is larger than this threshold, the point is counted as a force change point.
      'openif_nforce_threshold': 20,  #Threshold of number of force changing points to open the gripper.
      'openif_dw_grip': 0.02,  #Displacement of gripper movement.
    }
  if os.path.exists(config['FV_CTRL_CONFIG']):
    InsertDict(ctrl_config, LoadYAML(config['FV_CTRL_CONFIG']))
  def UpdateCtrlConfig(name, value):
    ctrl_config[name]= value
    SaveYAML(ctrl_config, config['FV_CTRL_CONFIG'], interactive=False)
  def AddCtrlConfigSliderWidget(widgets, name, prange):
    widgets['slider_ctrl_config_{}'.format(name)]= (
      'sliderh',{
        'range': prange,
        'value': ctrl_config[name],
        'n_labels': 3,
        'slider_style':1,
        'font_size_range': (10,12),
        #'size_policy': ('minimum', 'minimum'),
        'onvaluechange': lambda w,obj:UpdateCtrlConfig(name,obj.value())} )
    widgets['label_ctrl_config_{}'.format(name)]= (
      'label',{
        'text': name,
        'font_size_range': (12,14),
        'size_policy': ('minimum', 'minimum')} )
  def CtrlConfigSliderLayout(name):
    return ('label_ctrl_config_{}'.format(name),'slider_ctrl_config_{}'.format(name))
    #return ('boxv',None, ('label_ctrl_config_{}'.format(name),'slider_ctrl_config_{}'.format(name)) ),


  widgets_common= {
    'rviz': (
      'rviz',{
        'config': RVIZ_CONFIG,
        'size_policy': ('expanding', 'expanding')}),
    'spacer_cmn1': ('spacer', {
        'w': 400,
        'h': 1,
        'size_policy': ('fixed', 'fixed')      }),
    }

  widgets_init= {
    'label_init': (
      'label',{
        'text': 'Setup: ',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')}),
    'btn_init1': (
      'buttonchk',{
        'text':('(1)FV','(1)Stop FV'),
        'onclick':(lambda w,obj:(
                      run_cmd('fvp'),
                      rospy.sleep(0.2),
                      run_cmd('config_fv_l'),
                      run_cmd('config_fv_r'),
                      w.widgets['rviz'].setup(),
                      w.widgets['btn_init2'].setEnabled(True),
                     ),
                   lambda w,obj:(
                      stop_cmd('fvp'),
                      w.widgets['btn_init2'].setEnabled(False),
                     ) )}),
    'btn_init2': (
      'buttonchk',{
        'text':('(2)Gripper ctrl','(2)Stop gripper ctrl'),
        'enabled':False,
        'onclick':(lambda w,obj:(
                      run_cmd('fix_usb'),
                      run_cmd('gripper'),
                      run_cmd('joy'),
                      run_cmd('fv_gripper_ctrl'),
                      w.widgets['btn_init1'].setEnabled(False),
                     ),
                   lambda w,obj:(
                      stop_cmd('gripper'),
                      stop_cmd('joy'),
                      stop_cmd('fv_gripper_ctrl'),
                      w.widgets['btn_init1'].setEnabled(True),
                     ) )}),
    'btn_exit': (
      'button',{
        'text': 'Exit',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: w.close(), }),
    #'btn_shutdown_pc': (
      #'button',{
        #'text': 'Shutdown PC',
        #'onstatuschanged':lambda w,obj,status:(
                      #obj.setEnabled(status in (pm.NO_CORE_PROGRAM,pm.ROBOT_EMERGENCY_STOP)) ),
        #'size_policy': ('expanding', 'fixed'),
        #'onclick': lambda w,obj: run_cmd('shutdown_pc'), }),
    'btn_fv_record': (
      'buttonchk',{
        'text':('Record FV','Stop Record'),
        #'enabled':False,
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:(
                      run_cmd('start_record_l'),
                      run_cmd('start_record_r'),
                     ),
                   lambda w,obj:(
                      run_cmd('stop_record_l'),
                      run_cmd('stop_record_r'),
                     ) )}),
    }
  layout_init= (
    'grid',None,(
      ('label_init',0,0), ('btn_init1',0,1), ('btn_init2',0,2),
      ))

  widgets_joy= {
    'btn_grasp': (
      'button',{
        'text':'Grasp',
        #'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:set_joy('grasp_on',is_active=1),  }),
    'btn_hold': (
      'button',{
        'text':'Hold',
        #'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:set_joy('hold_on',is_active=1),   }),
    'btn_openif': (
      'button',{
        'text':'OpenIf',
        #'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:set_joy('openif_on',is_active=1),   }),
    'btn_stop': (
      'button',{
        'text':'Stop',
        #'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:(
                    set_joy('grasp_off'),
                    rospy.sleep(0.05),
                    set_joy('hold_off'),
                    rospy.sleep(0.05),
                    set_joy('openif_off'),
                    ),   }),
    'btn_grip_open': (
      'button',{
        'text': 'Open',
        #'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:(set_joy('open'),), }),
    'label_grip': (
      'label',{
        'text': 'Gripper',
        'size_policy': ('minimum', 'minimum')}),
    'joy_grip': (
      'virtual_joystick',{
        'kind':'hbox',
        'stick_color':[255,128,128],
        'size_policy': ('expanding','minimum'),
        'onstickmoved': lambda w,obj:set_joy('grip',obj.position(),is_active=1), }),
    }

  layout_joy= (
    'boxv',None, (
      ('boxh',None,('label_grip','joy_grip','btn_grip_open')),
      ('boxh',None, ('btn_grasp','btn_hold','btn_openif','btn_stop')),
      ))

  widgets_ctrl_config= {
    }
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'min_gstep', (0.0,0.01,0.0001))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'grasp_th',         (0,20,1))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'grasp_filter_len', (0,20,1))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'grasp_dstate_th',  (0,20,1))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_slip', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_oc', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_oo', (0.0,4.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_oa', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_slip', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_oc', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_oo', (0.0,4.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_oa', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_force', (0.0,4.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_nforce_threshold', (1,100,1))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_dw_grip', (0.0,0.1,0.001))

  layout_ctrl_config1= (
    'boxv',None, (
        ('boxh',None, CtrlConfigSliderLayout('min_gstep') ),
        ('boxh',None, CtrlConfigSliderLayout('grasp_th') ),
        ('boxh',None, CtrlConfigSliderLayout('grasp_filter_len') ),
        ('boxh',None, CtrlConfigSliderLayout('grasp_dstate_th') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_slip') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_oc') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_oo') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_oa') ),
      ))
  layout_ctrl_config2= (
    'boxv',None, (
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_slip') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_oc') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_oo') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_oa') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_force') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_nforce_threshold') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_dw_grip') ),
      ))

  widgets_debug= {
    'btn_rviz': (
      'buttonchk',{
        'text':('RViz','Stop RViz'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      run_cmd('rviz'),
                     ),
                   lambda w,obj:(
                      stop_cmd('rviz'),
                     ) )}),
    'label_processes': (
      'label',{
        'text': 'Processes: ',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')}),
    'combobox_procs': (
      'combobox',{
        'options':('(Process_name/PID)',),
        'font_size_range': (8,24),
        'size_adjust_policy': 'all_contents',
        'onactivated': lambda w,obj:None}),
    'btn_update_proc_list': (
      'button',{
        'text': 'Update',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: (UpdateProcList(pm,w.widgets['combobox_procs']),
                                  #w.widgets['combobox_procs'].resize(w.widgets['combobox_procs'].sizeHint())
                                  ) }),
    'btn_terminate_proc': (
      'button',{
        'text': 'Terminate',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.TerminateBGProcess(str(w.widgets['combobox_procs'].currentText()).split('/')[0]), }),
    'btn_kill_proc': (
      'button',{
        'text': 'Kill',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'fixed'),
        'onclick': lambda w,obj: pm.KillBGProcess(str(w.widgets['combobox_procs'].currentText()).split('/')[0]), }),
    'btn_dbg_exit': (
      'button',{
        'text': 'Exit',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: w.close(), }),
    }
  layout_debug= (
    'boxv',None,(
      ('boxv',None, ('btn_rviz',)),
      ('boxh',None, ('label_processes', ('boxv',None, (
                        'combobox_procs',
                        ('boxh',None, ('btn_update_proc_list','btn_terminate_proc','btn_kill_proc')),
                        ('boxh',None, ('btn_dbg_exit',)),
                        ))
                     )),
      ))

  layout_main= (
    'boxh',None, (
      'rviz',
      ('boxv',None, (
        ('tab','maintab',(
          #('Initialize',layout_init),
          #('Joy',layout_joy),
          ('Main',('boxv',None,(
            layout_init,
            'btn_fv_record',
            layout_joy,
            'btn_exit',
            ))),
          ('Config/1',layout_ctrl_config1),
          ('Config/2',layout_ctrl_config2),
          ('Debug',layout_debug),
          )),
        'spacer_cmn1')),
      ))

  app= InitPanelApp()
  win_size= (1000,500)
  if fullscreen:  #NOTE: fullscreen mode will work only with Qt5.
    print 'Screen size:', app.screens()[0].size()
    screen_size= app.screens()[0].size()
    win_size= (screen_size.width(),screen_size.height())
  panel= TSimplePanel('Robot Operation Panel', size=win_size, font_height_scale=300.0)
  panel.AddWidgets(widgets_common)
  panel.AddWidgets(widgets_init)
  panel.AddWidgets(widgets_joy)
  panel.AddWidgets(widgets_ctrl_config)
  panel.AddWidgets(widgets_debug)
  panel.Construct(layout_main)
  #for tab in panel.layouts['maintab'].tab:
    #tab.setFont(QtGui.QFont('', 24))
  panel.layouts['maintab'].tabs.setFont(QtGui.QFont('', 18))
  #panel.showFullScreen()
  #panel.showMaximized()

  #ROS system initialization.
  run_cmd('roscore')
  pm.InitNode()
  rospy.wait_for_service('/rosout/get_loggers', timeout=5.0)
  run_cmd('fix_usb')
  pm.StartVirtualJoyStick()
  panel.close_callback= lambda event: (
      pm.TerminateAllBGProcesses(),  #including roscore
      #stop_cmd('roscore'),
      True)[-1]

  RunPanelApp()
