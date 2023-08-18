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
    $ ln -s `rospack find ay_fv_extra`/config/fvp_5_l.yaml fvp300x_l.yaml
    $ ln -s `rospack find ay_fv_extra`/config/fvp_5_r.yaml fvp300x_r.yaml
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
from topic_monitor import TTopicMonitor
import std_msgs.msg

class TSubProcManagerJoy(QtCore.QObject, TSubProcManager, TJoyEmulator, TTopicMonitor):
  ontopicshzupdated= QtCore.pyqtSignal()

  def __init__(self, node_name='ctrl_panel', topics_to_monitor=None):
    QtCore.QObject.__init__(self)
    TSubProcManager.__init__(self)
    TJoyEmulator.__init__(self)
    self.node_name= node_name

    TTopicMonitor.__init__(self, topics_to_monitor)
    self.thread_topics_hz_callback= lambda: self.ontopicshzupdated.emit()

  def InitNode(self):
    rospy.init_node(self.node_name)
    rospy.sleep(0.1)

  def Setup(self):
    rospy.wait_for_service('/rosout/get_loggers', timeout=5.0)

    self.StartTopicMonitorThread()
    self.StartVirtualJoyStick()

    self.sub= {}
    for (topic,msg_type) in (
        ('gripper_pos',std_msgs.msg.Float64),
        ('target_pos',std_msgs.msg.Float64),
        ('active_script',std_msgs.msg.String), ):
      setattr(self, topic, None)
      self.sub[topic]= rospy.Subscriber('/fv_gripper_ctrl/{}'.format(topic), msg_type, lambda msg,topic=topic:self.TopicCallback(topic,msg))

  def Cleanup(self):
    self.StopTopicMonitorThread()
    for key,sub in self.sub.iteritems():
      sub.unregister()

  def TopicCallback(self, topic, msg):
    setattr(self, topic, msg.data)


def UpdateProcList(pm,combobox):
  combobox.clear()
  for name,proc in pm.procs.iteritems():
    combobox.addItem('{0}/{1}'.format(name,proc.pid))

if __name__=='__main__':
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  gripper_type= get_arg('-gripper_type=',get_arg('--gripper_type=','RHP12RNAGripper'))
  joy_dev= get_arg('-joy_dev=',get_arg('--joy_dev=','js0'))
  dxl_dev= get_arg('-dxl_dev=',get_arg('--dxl_dev=','USB0'))
  fullscreen= True if '-fullscreen' in sys.argv or '--fullscreen' in sys.argv else False
  #Gripper simulation flag:
  is_gsim= True if '-gsim' in sys.argv or '--gsim' in sys.argv else False
  #FV simulation flag:
  is_fvsim= True if '-fvsim' in sys.argv or '--fvsim' in sys.argv else False
  with_modbus= True if '-modbus' in sys.argv or '--modbus' in sys.argv else False

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
    'IS_GSIM': is_gsim,
    'IS_FVSIM': is_fvsim,
    'PLOT_LIST':[
        ('fv.area','area',1,None),
        ('fv.center','center_x',1,0),
        ('fv.center','center_y',1,1),
        ('fv.d_area','d_area',1,None),
        ('fv.d_center_norm','d_center_norm',1,None),
        ('fv.d_center','d_center_x',1,0),
        ('fv.d_center','d_center_y',1,1),
        ('fv.d_orientation','d_orientation',1,None),
        #('fv.normal_force','normal_force',
        ('fv.num_force_change','num_force_change',1,None),
        ('fv.orientation','orientation',1,None),
        ('fv.slip','slip',1,None),
        ('gripper_pos','gpos',2,None),
        ('target_pos','gpos_trg',2,None),
      ]
    }

  #List of commands (name: [[command/args],'fg'/'bg']).
  cmds= {
    'roscore': ['roscore','bg'],
    'fix_usb': ['sudo /sbin/fix_usb_latency.sh tty{DxlUSB}','fg'],
    'gripper': ['roslaunch ay_util gripper_selector.launch gripper_type:={GripperType} dxldev:=/dev/tty{DxlUSB} is_sim:={IS_GSIM}','bg'],
    'reboot_dxlg': ['rosrun ay_util dxlg_reboot.py /dev/tty{DxlUSB} {GripperType} Reboot','fg'],
    'factory_reset_dxlg': ['rosrun ay_util dxlg_reboot.py /dev/tty{DxlUSB} {GripperType} FactoryReset','fg'],
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
    'fv_gripper_ctrl': ['rosrun fv_gripper_ctrl fv_gripper_ctrl.py _gripper_type:={GripperType} _is_sim:=False','bg'],
    'modbus_port_fwd': ['sudo iptables -t nat -A PREROUTING -p tcp --dport 502 -j REDIRECT --to-ports 5020','fg'],
    'modbus_server': ['/sbin/fvgripper_modbus_srv.sh','bg'],
    'fvsignal_plot': ['rosrun fv_gripper_ctrl fvsignal_plot.py','bg'],
    }
  if is_gsim:
    for c in ('fix_usb','reboot_dxlg','factory_reset_dxlg'):
      cmds[c][1]= None
  if is_fvsim:
    cmds['fvp']= cmds['fvp_file']
    for c in ('config_fv_l','config_fv_r'):
      cmds[c][1]= None
  for key in cmds.iterkeys():
    if isinstance(cmds[key][0],str):
      cmds[key][0]= cmds[key][0].format(**config).split(' ')
  print config

  #List of topics to monitor the status:
  topics_to_monitor= {
    'Gripper': '/gripper_driver/joint_states',
    'FV_L': '/fingervision/fvp_1_l/prox_vision',
    'FV_R': '/fingervision/fvp_1_r/prox_vision',
    #'ModbusSrv': '/fingervision/fvp_1_r/prox_vision',
    }

  pm= TSubProcManagerJoy(topics_to_monitor=topics_to_monitor)
  run_cmd= lambda name: pm.RunBGProcess(name,cmds[name][0]) if cmds[name][1]=='bg' else\
                        pm.RunFGProcess(cmds[name][0]) if cmds[name][1]=='fg' else\
                        None
  stop_cmd= lambda name: pm.TerminateBGProcess(name)


  set_joy= lambda kind,value=None,is_active=0: pm.SetJoy(kind,value,is_active)


  status_grid_list_text= [
      dict(label='GPos', type='text', state='N/A'),
      dict(label='GPosTrg', type='text', state='N/A'),
      dict(label='Action', type='text', state='N/A'),
    ]
  status_grid_list_color= [dict(label=key, type='color', state='red') for key in sorted(pm.topics_to_monitor.iterkeys())]
  def UpdateStatusGridText(w,obj,status=None):
    obj.UpdateStatus('GPos', '{:.5f} [m]'.format(pm.gripper_pos) if pm.gripper_pos is not None else 'N/A')
    obj.UpdateStatus('GPosTrg', '{:.5f} [m]'.format(pm.target_pos) if pm.target_pos is not None else 'N/A')
    obj.UpdateStatus('Action', '(move-to)' if pm.active_script=='' else pm.active_script if pm.active_script is not None else 'N/A')
  def UpdateStatusGridColor(w,obj,status=None):
    for key,topic in pm.topics_to_monitor.iteritems():
      obj.UpdateStatus(key, 'green' if pm.IsActive(key) else 'red')

  #UI for configuring FV control parameters:
  ctrl_config= {
      #Common control parameters:
      'min_gstep': 0.001,  #Minimum gripper step size.
      'force_change_sensitivity': 1.3,  #Sensitivity of each force element; if the norm of force change is larger than this threshold, the point is counted as a force change point.
      'force_init_len': 10,  #Length of initial force array to average as a basis to compute the force array change.
      #Parameters used in fv.grasp:
      'grasp_nforce_threshold': 10,  #Threshold of number of force changing points to stop closing the gripper.
      #Parameters used in fv.hold, fv.pickup2a, fv.pickup2b:
      'hold_sensitivity_slip': 0.02,  #Sensitivity of slip detection (smaller is more sensitive).
      'hold_sensitivity_oc': 2.0,  #Sensitivity of object-center-movement detection (smaller is more sensitive).
      'hold_sensitivity_oa': 2.0,  #Sensitivity of object-area-change detection (smaller is more sensitive).
      #Parameters used in fv.openif:
      'openif_sensitivity_slip': 0.025,  #Sensitivity of slip detection (smaller is more sensitive).
      'openif_sensitivity_oc': 2.0,  #Sensitivity of object-center-movement detection (smaller is more sensitive).
      'openif_sensitivity_oa': 2.0,  #Sensitivity of object-area-change detection (smaller is more sensitive).
      'openif_nforce_threshold': 14,  #Threshold of number of force changing points to open the gripper.
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
    'spacer_cmn2': ('spacer', {
        'w': 100,
        'h': 1,
        'size_policy': ('fixed', 'expanding')      }),
    'status_grid_text': (
      'status_grid',{
        'list_status': status_grid_list_text,
        'direction':'vertical',
        'shape':'square',
        'margin':(0.05,0.05),
        'rows':None,
        'columns':1,
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum'),
        'ontopicshzupdated': UpdateStatusGridText,
        }),
    'status_grid_color': (
      'status_grid',{
        'list_status': status_grid_list_color,
        'direction':'horizontal',
        'shape':'square',
        'margin':(0.05,0.05),
        'rows':None,
        'columns':3,
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum'),
        'ontopicshzupdated': UpdateStatusGridColor,
        }),
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
                      run_cmd('reboot_dxlg'),
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
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'force_change_sensitivity', (0.0,4.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'force_init_len', (0,100,1))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'grasp_nforce_threshold', (0,100,1))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_slip', (0.0,1.0,0.001))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_oc', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_oa', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_slip', (0.0,1.0,0.001))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_oc', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_oa', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_nforce_threshold', (1,100,1))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_dw_grip', (0.0,0.1,0.001))

  layout_ctrl_config1= (
    'boxv',None, (
        ('boxh',None, CtrlConfigSliderLayout('min_gstep') ),
        ('boxh',None, CtrlConfigSliderLayout('force_change_sensitivity') ),
        ('boxh',None, CtrlConfigSliderLayout('force_init_len') ),
        ('boxh',None, CtrlConfigSliderLayout('grasp_nforce_threshold') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_slip') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_oc') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_oa') ),
        'spacer_cmn2',
      ))
  layout_ctrl_config2= (
    'boxv',None, (
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_slip') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_oc') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_oa') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_nforce_threshold') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_dw_grip') ),
        'spacer_cmn2',
      ))


  def RunFVSignalPlot(w,obj):
    plot_list= [p for p in config['PLOT_LIST'] if w.widgets['checkbox_{}'.format(p[1])].isChecked()]
    if len(plot_list)==0:  return
    cmd= cmds['fvsignal_plot'][0]+['--plots={}'.format(repr(plot_list).replace(' ',''))]
    print 'fvsignal_plot:',cmd
    pm.RunBGProcess('fvsignal_plot',cmd)
  widgets_plot_cbs= {
    'checkbox_{}'.format(plot_label): ('checkbox',{'text': plot_label})
      for (signal_name,plot_label,axis,index) in config['PLOT_LIST']}
  layout_plot_cbs= (
    'boxv',None,(
        'checkbox_{}'.format(plot_label) for (signal_name,plot_label,axis,index) in config['PLOT_LIST']
      ))
  widgets_plots= {
    'btn_plot': (
      'buttonchk',{
        'text':('Plot','Stop Plot'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      RunFVSignalPlot(w,obj),
                     ),
                   lambda w,obj:(
                      stop_cmd('fvsignal_plot'),
                     ) )}),
    }
  layout_plots= (
    'boxh',None,(
      ('boxv',None, (layout_plot_cbs,'spacer_cmn2')),
      ('boxv',None, ('btn_plot','spacer_cmn2')),
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
    'btn_modbus_server': (
      'buttonchk',{
        'text':('Modbus Server','Stop Modbus Srv'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      run_cmd('modbus_port_fwd'),
                      run_cmd('modbus_server'),
                     ),
                   lambda w,obj:(
                      stop_cmd('modbus_server'),
                     ) )}),
    'label_dxlg': (
      'label',{
        'text': 'Gripper: ',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')}),
    'btn_dxlg_reboot': (
      'button',{
        'text': 'Reboot',
        'font_size_range': (8,24),
        #'size_policy': ('minimum', 'minimum'),
        'onclick': lambda w,obj: run_cmd('reboot_dxlg'), }),
    'btn_dxlg_factory_reset': (
      'button',{
        'text': 'FactoryReset',
        'font_size_range': (8,24),
        #'size_policy': ('minimum', 'minimum'),
        'onclick': lambda w,obj: run_cmd('factory_reset_dxlg'), }),
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
  if not with_modbus:  del widgets_debug['btn_modbus_server']
  layout_debug= (
    'boxv',None,(
      ('boxv',None, ('btn_rviz',)),
      ('boxv',None, ('btn_modbus_server',) if with_modbus else ()),
      ('boxh',None, ('label_dxlg', ('boxv',None, (
                        ('boxh',None, ('btn_dxlg_reboot','btn_dxlg_factory_reset')),
                        ))
                     )),
      ('boxh',None, ('label_processes', ('boxv',None, (
                        'combobox_procs',
                        ('boxh',None, ('btn_update_proc_list','btn_terminate_proc','btn_kill_proc')),
                        ('boxh',None, ('btn_dbg_exit',)),
                        ))
                     )),
      ))

  layout_main= (
    'boxh',None, (
      #'rviz',
      ('boxv',None, (
        'rviz',
        ('boxv',None, ('status_grid_text','status_grid_color')),
        )),
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
          ('Plot',layout_plots),
          ('Advanced',layout_debug),
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
  panel.AddWidgets(widgets_plot_cbs)
  panel.AddWidgets(widgets_plots)
  panel.AddWidgets(widgets_debug)
  panel.Construct(layout_main)
  #for tab in panel.layouts['maintab'].tab:
    #tab.setFont(QtGui.QFont('', 24))
  panel.layouts['maintab'].tabs.setFont(QtGui.QFont('', 18))
  #panel.showFullScreen()
  #panel.showMaximized()

  #Since the ontopicshzupdated signal is emitted from ProcessManager,
  #we connect the ontopicshzupdated slots of panel to it.
  for w_name, (w_type, w_param) in panel.widgets_in.iteritems():
    if 'ontopicshzupdated' in w_param and w_param['ontopicshzupdated'] is not None:
      pm.ontopicshzupdated.connect(lambda w_param=w_param,w_name=w_name: w_param['ontopicshzupdated'](panel,panel.widgets[w_name]))

  #ROS system initialization.
  run_cmd('roscore')
  pm.InitNode()
  pm.Setup()
  run_cmd('fix_usb')
  panel.close_callback= lambda event: (
      pm.TerminateAllBGProcesses(),  #including roscore
      #stop_cmd('roscore'),
      pm.Cleanup(),
      True)[-1]

  RunPanelApp()
