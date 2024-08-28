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
from __future__ import print_function
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import os,sys
import subprocess
import rospy
import rospkg
from ay_py.core import InsertDict, LoadYAML, SaveYAML, CPrint
from ay_py.tool.py_panel import TSimplePanel, InitPanelApp, RunPanelApp, AskYesNoDialog, QtCore, QtGui
sys.path.append(os.path.join(rospkg.RosPack().get_path('ay_util'),'scripts'))
from proc_manager import TSubProcManager
from joy_fv import TJoyEmulator
from topic_monitor import TTopicMonitor
import std_msgs.msg
import fv_sensor
import numpy as np

class TSubProcManagerJoy(QtCore.QObject, TSubProcManager, TJoyEmulator, TTopicMonitor):
  ontopicshzupdated= QtCore.pyqtSignal()

  def __init__(self, node_name='ctrl_panel', topics_to_monitor=None):
    QtCore.QObject.__init__(self)
    TSubProcManager.__init__(self)
    TJoyEmulator.__init__(self)
    self.flag_relaunch= False
    self.node_name= node_name
    self.fv= fv_sensor.TFVSensor()  #FV sensor module to access the FV services.
    self.pub= {}
    self.sub= {}

    TTopicMonitor.__init__(self, topics_to_monitor)
    self.thread_topics_hz_callback= lambda: self.ontopicshzupdated.emit()

  def InitNode(self):
    rospy.init_node(self.node_name)
    rospy.sleep(0.1)

  def Setup(self):
    rospy.wait_for_service('/rosout/get_loggers', timeout=5.0)

    self.StartTopicMonitorThread()
    self.StartVirtualJoyStick()

    for (topic,msg_type) in (
        ('gripper_pos',std_msgs.msg.Float64),
        ('target_pos',std_msgs.msg.Float64),
        ('active_script',std_msgs.msg.String), ):
      setattr(self, topic, None)
      self.sub[topic]= rospy.Subscriber('/fv_gripper_ctrl/{}'.format(topic), msg_type, lambda msg,topic=topic:self.TopicCallback(topic,msg))
    self.r_pxv_obj_detection= None
    self.l_pxv_obj_detection= None
    self.r_blob_fps= None
    self.l_blob_fps= None
    self.r_pxv_fps= None
    self.l_pxv_fps= None
    self.sub['r_pxv_obj_detection']= rospy.Subscriber('/fingervision/fvp_1_r/pxv_obj_detection', std_msgs.msg.Bool, lambda msg:self.TopicCallback('r_pxv_obj_detection',msg))
    self.sub['l_pxv_obj_detection']= rospy.Subscriber('/fingervision/fvp_1_l/pxv_obj_detection', std_msgs.msg.Bool, lambda msg:self.TopicCallback('l_pxv_obj_detection',msg))
    self.sub['r_blob_fps']= rospy.Subscriber('/fingervision/fvp_1_r/blob_fps', std_msgs.msg.Float32, lambda msg:self.TopicCallback('r_blob_fps',msg))
    self.sub['l_blob_fps']= rospy.Subscriber('/fingervision/fvp_1_l/blob_fps', std_msgs.msg.Float32, lambda msg:self.TopicCallback('l_blob_fps',msg))
    self.sub['r_pxv_fps']= rospy.Subscriber('/fingervision/fvp_1_r/pxv_fps', std_msgs.msg.Float32, lambda msg:self.TopicCallback('r_pxv_fps',msg))
    self.sub['l_pxv_fps']= rospy.Subscriber('/fingervision/fvp_1_l/pxv_fps', std_msgs.msg.Float32, lambda msg:self.TopicCallback('l_pxv_fps',msg))

  def SerupFVSrv(self, fv_names, fv_node_names=None):
    self.fv.Setup(gripper=None, g_param=None, frame_id='base_link',
                  fv_names=fv_names, node_names=fv_node_names, service_only=True)

  def StopFVSrv(self):
    self.fv.Stop()

  def SetupGripper(self):
    self.pub['set_target_pos']= rospy.Publisher('/fv_gripper_ctrl/set_target_pos', std_msgs.msg.Float64, queue_size=1)

  def StopGripper(self):
    pass

  def Cleanup(self):
    self.StopGripper()
    self.fv.Cleanup()
    self.StopTopicMonitorThread()
    for key,sub in self.sub.iteritems():
      sub.unregister()

  def TopicCallback(self, topic, msg):
    setattr(self, topic, msg.data)


def UpdateProcList(pm,combobox):
  combobox.clear()
  for name,proc in pm.procs.iteritems():
    combobox.addItem('{0}/{1}'.format(name,proc.pid))

#Align windows to window_positions (dict of: {'window_title': 'x,y,w,h'}).
def AlignWindows(window_positions):
  try:
    window_list= subprocess.check_output(['wmctrl', '-l']).decode('utf-8').splitlines()
  except subprocess.CalledProcessError:
    CPrint(4, 'Failed to get the window list with wmctrl.')
    CPrint('Please check if wmctrl is installed.')
    return

  def get_window_id_by_title(window_list, title):
    for line in window_list:
      if title.lower() in line.lower():
        return line.split()[0]
    return None

  for title, pos in window_positions.items():
    window_id= get_window_id_by_title(window_list, title)
    if window_id:
      subprocess.call(['wmctrl', '-i', '-r', window_id, '-e', '0,{}'.format(pos)])
      subprocess.call(['wmctrl', '-i', '-a', window_id])
    else:
      print('No window with title containing "{}" found.'.format(title))

def RelaunchProgram():
  print("Relaunching program...")
  python= sys.executable
  os.execl(python, python, *sys.argv)

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
  #Sensor-only application:
  sensor_app= True if '-sensor_app' in sys.argv or '--sensor_app' in sys.argv else False
  with_modbus= True if '-modbus' in sys.argv or '--modbus' in sys.argv else False

  HOME= os.environ['HOME']
  DATA_DIR= HOME+'/data'
  CONFIG_DIR= DATA_DIR+'/config'
  TEMP_DIR= '/tmp'
  RVIZ_CONFIG= HOME+'/.rviz/default.rviz'
  UI_CONFIG_FILE= CONFIG_DIR+'/fv_ctrl_ui.yaml'
  #PANEL_CONFIG_FILE= CONFIG_DIR+'/fv_ctrl_panel.yaml'
  RIGHT,LEFT= fv_sensor.RIGHT,fv_sensor.LEFT

  if sensor_app:
    is_gsim= True

  #Search for config files (YAML files that have a CONFIG_NAME field).
  config_file_list= [('(Default Config)', '')]  #List of (config name, file name).
  for f in os.listdir(CONFIG_DIR):
    if f.endswith('.yaml') and not os.path.isdir(os.path.join(CONFIG_DIR,f)):
      try:
        d= LoadYAML(os.path.join(CONFIG_DIR,f))
        if 'CONFIG_NAME' in d:
          if d['CONFIG_NAME'] in dict(config_file_list):
            print('WARNING: Multiple config files with the same CONFIG_NAME value.')
            print('  File 1: {}: {}'.format(d['CONFIG_NAME'], dict(config_file_list)[d['CONFIG_NAME']]))
            print('  File 2: {}: {}'.format(d['CONFIG_NAME'], f))
            config_file_list= [(config_name, file_name) if config_name!=d['CONFIG_NAME']
                               else (config_name, f)
                                  for config_name, file_name in config_file_list]
          else:
            config_file_list.append((d['CONFIG_NAME'], f))
      except Exception:
        pass
  for config_name, file_name in config_file_list:
    print('Config file option: {}: {}'.format(config_name, file_name))
  print('')

  #Parameters managed by UI (UI can change those values):
  ui_config={
    'MODBUS_C_SRV': '10.10.6.204',
    'MODBUS_C_ROBOT': 'Yaskawa',
    'CONFIG_FILE': 'fv_ctrl_panel.yaml',
    }
  #Loading UI config from a file.
  if os.path.exists(UI_CONFIG_FILE):
    InsertDict(ui_config, LoadYAML(UI_CONFIG_FILE))
  def DumpUIConfigToFile():
    SaveYAML(ui_config, UI_CONFIG_FILE, interactive=False)
  def UpdateUIConfig(name, value):
    ui_config[name]= value
    DumpUIConfigToFile()
  DumpUIConfigToFile()

  #Search for ui_config['CONFIG_FILE'] in config_file_list.
  config_file_index= 0
  index= 0
  for config_name, file_name in config_file_list:
    if ui_config['CONFIG_FILE']==file_name:
      config_file_index= index
      break
    index+= 1

  #Control Panel Parameters:
  config={
    'CONFIG_NAME': 'Default Config',
    'GRIPPER_TYPE': gripper_type,
    'JOY_USB': joy_dev,
    'DXL_USB': dxl_dev,
    'IOLINK_MODBUS_URI': '10.10.6.207',
    'FV_L_DEV': '/media/video_fv1',
    'FV_R_DEV': '/media/video_fv2',
    'FV_BASE_DIR': DATA_DIR,
    'FV_L_CONFIG': 'config/fvp300x_l.yaml',
    'FV_R_CONFIG': 'config/fvp300x_r.yaml',
    'FV_L_GUI_CONFIG': 'data_gen/fvp_l.yaml',
    'FV_R_GUI_CONFIG': 'data_gen/fvp_r.yaml',
    'FV_FILE_BASE_DIR': subprocess.check_output('rospack find ay_fv_extra'.split(' ')).strip(),
    'FV_FILE_L_CONFIG': 'config/fvp_file1_l.yaml',
    'FV_FILE_R_CONFIG': 'config/fvp_file1_r.yaml',
    'FV_FILE_L_GUI_CONFIG': 'data_gen/fvp_file1_l.yaml',
    'FV_FILE_R_GUI_CONFIG': 'data_gen/fvp_file1_r.yaml',
    'FV_CTRL_CONFIG': CONFIG_DIR+'/fv_ctrl.yaml',
    'FV_NAMES': {RIGHT:'fvp_1_r',LEFT:'fvp_1_l'},
    'VIDEO_PREFIX': DATA_DIR+'/data_gen/video-',
    'LOG_PREFIX': DATA_DIR+'/data_gen/log-',
    'IS_GSIM': is_gsim,
    'IS_FVSIM': is_fvsim,
    'SENSOR_APP': sensor_app,
    'PLOT_LOGGER_CONFIG': CONFIG_DIR+'/plot_logger.yaml',
    'WINDOW_ALIGNMENT':{
        'fvp_1_l-blob': '0,0,640,480',
        'fvp_1_r-blob': '648,0,640,480',
        'fvp_1_l-pxv': '0,545,640,480',
        'fvp_1_r-pxv': '648,545,640,480',
        'Robot Operation Panel': '1200,0,600,500',
      },
    'MODBUS_CONFIG_TEMPORARY': TEMP_DIR+'/fv_ctrl_modbus_tmp.yaml',
    'MODBUS_PROTOCOL_CONFIG': CONFIG_DIR+'/fv_ctrl_modbus_protocol.yaml',
    }
  config['FV_NAMES_STR']= '{{{}}}'.format(','.join("'{}':'{}'".format(key,value) for key,value in config['FV_NAMES'].iteritems()))

  #Loading config from a file.
  SaveYAML(config, TEMP_DIR+'/fv_ctrl_panel_default.yaml', interactive=False)
  ctrl_panel_config_file= os.path.join(CONFIG_DIR,ui_config['CONFIG_FILE'])
  if os.path.exists(ctrl_panel_config_file) and not os.path.isdir(ctrl_panel_config_file):
    InsertDict(config, LoadYAML(ctrl_panel_config_file))
  SaveYAML(config, TEMP_DIR+'/fv_ctrl_panel_latest.yaml', interactive=False)
  print(config)

  '''
  #Parameters managed by UI:
  ui_managed_param={
    'MODBUS_C_SRV': '10.10.6.204',
    'MODBUS_C_PORT': 502,
    'MODBUS_C_ROBOT': 'Yaskawa',
    }
  if os.path.exists(config['PARAM_FILE_UI']):
    InsertDict(ui_managed_param, LoadYAML(config['PARAM_FILE_UI']))
  def GenerateParamFile():
    SaveYAML(ui_managed_param, config['PARAM_FILE_UI'], interactive=False)
  def UpdateParam(name, value):
    ui_managed_param[name]= value
    GenerateParamFile()
  GenerateParamFile()
  '''


  #Modbus config
  modbus_config= dict()
  #Copy UI config to Modbus config and optionally save config to a temporary file.
  def MapUIConfigToModbusConfig(save_to_temp=True):
    modbus_config={
      'MODBUS_C_SRV': ui_config['MODBUS_C_SRV'],
      'MODBUS_C_ROBOT': ui_config['MODBUS_C_SRV'],
      'MODBUS_C_PORT': 502,
      }
    if save_to_temp:
      SaveYAML(modbus_config, config['MODBUS_CONFIG_TEMPORARY'], interactive=False)
  MapUIConfigToModbusConfig()


  #List of commands (name: [[command/args],'fg'/'bg']).
  cmds= {
    'roscore': ['roscore','bg'],
    'fix_usb': ['sudo /sbin/fix_usb_latency.sh tty{DXL_USB}','fg'],
    'gripper':           ['roslaunch ay_util gripper_selector.launch gripper_type:={GRIPPER_TYPE} dxldev:=/dev/tty{DXL_USB} is_sim:={IS_GSIM}','bg'],
    'gripper_geh':       ['/sbin/geh6000il_cpsl08p1en_driver.py _gripper_type:={GRIPPER_TYPE} _modbus_uri:={IOLINK_MODBUS_URI} _is_sim:={IS_GSIM} _is_test:=False','bg'],
    'reboot_dxlg':       ['roslaunch ay_util gripper_reboot.launch gripper_type:={GRIPPER_TYPE} dxldev:=/dev/tty{DXL_USB} command:=Reboot','fg'],
    'factory_reset_dxlg':['roslaunch ay_util gripper_reboot.launch gripper_type:={GRIPPER_TYPE} dxldev:=/dev/tty{DXL_USB} command:=FactoryReset','fg'],
    'joy': ['rosrun joy joy_node joy_node {JOY_USB}','bg'],
    'fvp': ['roslaunch fingervision fvp_general.launch pkg_dir:={FV_BASE_DIR} config1:={FV_L_CONFIG},{FV_L_GUI_CONFIG} config2:={FV_R_CONFIG},{FV_R_GUI_CONFIG}','bg'],
    'fvp_file': ['roslaunch fingervision fvp_general.launch pkg_dir:={FV_FILE_BASE_DIR} config1:={FV_FILE_L_CONFIG},{FV_FILE_L_GUI_CONFIG} config2:={FV_FILE_R_CONFIG},{FV_FILE_R_GUI_CONFIG}','bg'],
    'config_fv_l': ['rosrun fingervision conf_cam2.py {FV_L_DEV} file:CameraParams:0:{FV_BASE_DIR}/{FV_L_CONFIG}','fg'],
    'config_fv_r': ['rosrun fingervision conf_cam2.py {FV_R_DEV} file:CameraParams:0:{FV_BASE_DIR}/{FV_R_CONFIG}','fg'],
    'gui_config_fv_l': ['rosrun fingervision conf_cam_gui.py {FV_L_DEV}','bg'],
    'gui_config_fv_r': ['rosrun fingervision conf_cam_gui.py {FV_R_DEV}','bg'],
    #'start_record_l': ['rosservice call /fingervision/fvp_1_l/start_record','fg'],
    #'start_record_r': ['rosservice call /fingervision/fvp_1_r/start_record','fg'],
    #'stop_record_l': ['rosservice call /fingervision/fvp_1_l/stop_record','fg'],
    #'stop_record_r': ['rosservice call /fingervision/fvp_1_r/stop_record','fg'],
    'rviz': ['rosrun rviz rviz -d {0}'.format(RVIZ_CONFIG),'bg'],
    'fv_gripper_ctrl': ['rosrun fv_gripper_ctrl fv_gripper_ctrl.py _gripper_type:={GRIPPER_TYPE} _fv_names:={FV_NAMES_STR} _is_sim:=False','bg'],
    'fvsignal_plot': ['rosrun fv_gripper_ctrl fvsignal_plot.py --plots={PLOT_LOGGER_CONFIG}','bg'],
    'fvsignal_log': ['rosrun fv_gripper_ctrl fvsignal_log.py --logs={PLOT_LOGGER_CONFIG} --file_prefix={LOG_PREFIX}','bg'],
    'ln_cams': ['rosrun fingervision ln_cams.sh','fg'],
    'modbus_port_fwd': ['sudo iptables -t nat -A PREROUTING -p tcp --dport 502 -j REDIRECT --to-ports 5020','fg'],
    'modbus_server': ['/sbin/fvgripper_modbus_srv.sh --config_protocol={MODBUS_PROTOCOL_CONFIG}','bg'],
    'modbus_client': ['/sbin/fvgripper_modbus_client.py --config={MODBUS_CONFIG_TEMPORARY} --config_protocol={MODBUS_PROTOCOL_CONFIG}','bg'],
    'geh60_homing': ['rosservice call /gripper_driver/move Homing [0] [100] [100] 0','fg'],
    'geh60_deactivate': ['rosservice call /gripper_driver/move Deactivate [0] [100] [100] 0','fg'],
    'geh60_activate': ['rosservice call /gripper_driver/move Activate [0] [100] [100] 0','fg'],
    'geh60_startmoveth': ['rosservice call /gripper_driver/move StartMoveTh [0] [100] [100] 0','fg'],
    'geh60_stopmoveth': ['rosservice call /gripper_driver/move StopMoveTh [0] [100] [100] 0','fg'],
    }
  is_geh60= config['GRIPPER_TYPE'].startswith('GEH60')
  if is_gsim or is_geh60:
    for c in ('fix_usb','reboot_dxlg','factory_reset_dxlg'):
      cmds[c][1]= None
  if is_fvsim:
    cmds['fvp']= cmds['fvp_file']
    for c in ('config_fv_l','config_fv_r'):
      cmds[c][1]= None
    config['FV_L_GUI_CONFIG']= config['FV_FILE_L_GUI_CONFIG']
    config['FV_R_GUI_CONFIG']= config['FV_FILE_R_GUI_CONFIG']
  for key in cmds.iterkeys():
    if isinstance(cmds[key][0],str):
      cmds[key][0]= cmds[key][0].format(**config).split(' ')

  #List of topics to monitor the status:
  topics_to_monitor= {
    'FV_L': '/fingervision/fvp_1_l/prox_vision',
    'FV_R': '/fingervision/fvp_1_r/prox_vision',
    }
  if not sensor_app:
    topics_to_monitor['Gripper']= '/gripper_driver/joint_states'
  if with_modbus:
    topics_to_monitor['ModbusSrv']= '/fv_gripper_modbus_srv/status'
    topics_to_monitor['ModbusCli']= '/fv_gripper_modbus_client/status'

  pm= TSubProcManagerJoy(topics_to_monitor=topics_to_monitor)
  run_cmd= lambda name: pm.RunBGProcess(name,cmds[name][0]) if cmds[name][1]=='bg' else\
                        pm.RunFGProcess(cmds[name][0]) if cmds[name][1]=='fg' else\
                        None
  stop_cmd= lambda name: pm.TerminateBGProcess(name)


  set_joy= lambda kind,value=None,is_active=0: pm.SetJoy(kind,value,is_active)


  if not sensor_app:
    status_grid_list_text= [
        dict(label='ObjDetect(l,r)', type='text', state='N/A'),
        dict(label='FPS/blob(l,r)', type='text', state='N/A'),
        dict(label='FPS/pxv(l,r)', type='text', state='N/A'),
        dict(label='GPos', type='text', state='N/A'),
        dict(label='GPosTrg', type='text', state='N/A'),
        dict(label='Action', type='text', state='N/A'),
      ]
  else:
    status_grid_list_text= [
        dict(label='ObjDetect(l,r)', type='text', state='N/A'),
        dict(label='FPS/blob(l,r)', type='text', state='N/A'),
        dict(label='FPS/pxv(l,r)', type='text', state='N/A'),
      ]

  status_grid_list_color= [dict(label=key, type='color', state='red') for key in sorted(pm.topics_to_monitor.iterkeys())]
  def UpdateStatusGridText(w,obj,status=None):
    to_str_2= lambda x:'{:.2f}'.format(x) if x is not None else 'N/A'
    obj.UpdateStatus('ObjDetect(l,r)', '({},{})'.format(pm.l_pxv_obj_detection,pm.r_pxv_obj_detection))
    obj.UpdateStatus('FPS/blob(l,r)', '({},{})'.format(to_str_2(pm.l_blob_fps),to_str_2(pm.r_blob_fps)))
    obj.UpdateStatus('FPS/pxv(l,r)', '({},{})'.format(to_str_2(pm.l_pxv_fps),to_str_2(pm.r_pxv_fps)))
    if not sensor_app:
      obj.UpdateStatus('GPos', '{:.5f} [m]'.format(pm.gripper_pos) if pm.gripper_pos is not None else 'N/A')
      obj.UpdateStatus('GPosTrg', '{:.5f} [m]'.format(pm.target_pos) if pm.target_pos is not None else 'N/A')
      obj.UpdateStatus('Action', '(move-to)' if pm.active_script=='' else pm.active_script if pm.active_script is not None else 'N/A')
  def UpdateStatusGridColor(w,obj,status=None):
    for key,topic in pm.topics_to_monitor.iteritems():
      obj.UpdateStatus(key, 'green' if pm.IsActive(key) else 'red')

  #UI for configuring FV control parameters:
  ctrl_config= {
      #Basic configuration:
      'current_limit': None,  #Limit of the gripper operating current. Value depends on a gripper. null/None means using the gripper default. In RHP12RN case, the default is 1984 mA (max).
      'gpos_range': [None, None],  #Range (min,max) to limit the gripper motion (null/None for no limit).
      #Common control parameters:
      'min_gstep': 0.001,  #Minimum gripper step size.
      'min_obj_area': 0.05,  #Minimum object area to detect an object (used in center/orientation/da_area/d_center/da_orientation sensors).
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

  #Plot and logger config.
  #Each item=[signal name, label, axis (1 or 2), tuple of value-index (specify None for scalar), enabled]
  plot_logger_config= [
      ['fv.area_l','area_l',1,None, False],
      ['fv.area_r','area_r',1,None, False],
      ['fv.area','area',1,None, False],
      ['fv.center_l','center_l_x',1,0, False],
      ['fv.center_l','center_l_y',1,1, False],
      ['fv.center_r','center_r_x',1,0, False],
      ['fv.center_r','center_r_y',1,1, False],
      ['fv.d_center_l','d_center_l_x',1,0, False],
      ['fv.d_center_l','d_center_l_y',1,1, False],
      ['fv.d_center_r','d_center_r_x',1,0, False],
      ['fv.d_center_r','d_center_r_y',1,1, False],
      ['fv.da_area','da_area',1,None, False],
      ['fv.da_center_norm','da_center_norm',1,None, False],
      ['fv.da_orientation','da_orientation',1,None, False],
      ['fv.force_l','force_l_x',1,0, False],
      ['fv.force_l','force_l_y',1,1, False],
      ['fv.force_l','force_l_z',1,2, False],
      ['fv.force_r','force_r_x',1,0, False],
      ['fv.force_r','force_r_y',1,1, False],
      ['fv.force_r','force_r_z',1,2, False],
      ['fv.is_detected_l','is_detected_l',1,None, False],
      ['fv.is_detected_r','is_detected_r',1,None, False],
      ['fv.num_force_change','num_force_change',1,None, False],
      ['fv.orientation_l','orientation_l',1,None, False],
      ['fv.orientation_r','orientation_r',1,None, False],
      ['fv.slip_l','slip_l',1,None, False],
      ['fv.slip_r','slip_r',1,None, False],
      ['fv.slip','slip',1,None, False],
      ['gripper_pos','gpos',2,None, False],
      ['target_pos','gpos_trg',2,None, False],
    ]
  if os.path.exists(config['PLOT_LOGGER_CONFIG']):
    plot_logger_config_loaded= LoadYAML(config['PLOT_LOGGER_CONFIG'])
    #Add items in plot_logger_config to plot_logger_config_loaded if they are missing.
    labels_loaded= [label for (signal_name,label,axis,index,enabled) in plot_logger_config_loaded]
    for entry in plot_logger_config:
      if entry[1] not in labels_loaded:
        plot_logger_config_loaded.append(entry)
    #Ignore items in plot_logger_config_loaded if their labels are not included in plot_logger_config.
    labels_orig= [label for (signal_name,label,axis,index,enabled) in plot_logger_config]
    plot_logger_config= [[signal_name,label,axis,index,enabled]
                         for (signal_name,label,axis,index,enabled) in plot_logger_config_loaded
                         if label in labels_orig]
  else:
    SaveYAML(plot_logger_config, config['PLOT_LOGGER_CONFIG'], interactive=False)
  def UpdatePlotLoggerConfig(label, checked):
    try:
      entry= next(entry for entry in plot_logger_config if entry[1]==label)
      entry[-1]= checked
    except StopIteration:
      pass
    SaveYAML(plot_logger_config, config['PLOT_LOGGER_CONFIG'], interactive=False)

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
    'label_config': (
      'label',{
        'text': 'Config: ',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')}),
    'combo_config_file_list': (
      'combobox',{
        'options': [config_name for config_name, file_name in config_file_list],
        'index': config_file_index,
        'font_size_range': (8,24),
        'size_policy': ('expanding', 'minimum'),
        'ontextchanged': lambda w,obj:(
                            UpdateUIConfig('CONFIG_FILE',dict(config_file_list)[str(obj.currentText())]),
                            (setattr(pm,'flag_relaunch', True), w.close()) if AskYesNoDialog(w,'Restarting the program is needed to make the new configuration effective.\nDo you want to restart now?') else None
                            ) }),
    'btn_init1': (
      'buttonchk',{
        'text':('(1)FV','(1)Stop FV'),
        'onclick':(lambda w,obj:(
                      run_cmd('fvp'),
                      rospy.sleep(0.2),
                      run_cmd('config_fv_l'),
                      run_cmd('config_fv_r'),
                      pm.SerupFVSrv(fv_names=config['FV_NAMES']),
                      pm.fv.CallSrv('set_video_prefix', config['VIDEO_PREFIX']),
                      w.widgets['rviz'].setup(),
                      w.widgets['btn_init2'].setEnabled(True),
                      w.widgets['combo_config_file_list'].setEnabled(False),
                     ),
                   lambda w,obj:(
                      pm.StopFVSrv(),
                      stop_cmd('fvp'),
                      w.widgets['btn_init2'].setEnabled(False),
                      w.widgets['combo_config_file_list'].setEnabled(True),
                     ) )}),
    'btn_init2': (
      'buttonchk',{
        'text':('(2)Gripper ctrl','(2)Stop gripper ctrl') if not sensor_app
                else ('(2)Run filters','(2)Stop filters'),
        'enabled':False,
        'onclick':(lambda w,obj:(
                      run_cmd('reboot_dxlg'),
                      run_cmd('fix_usb'),
                      run_cmd('gripper' if not is_geh60 else 'gripper_geh'),
                      run_cmd('joy'),
                      run_cmd('fv_gripper_ctrl'),
                      pm.SetupGripper(),
                      w.widgets['btn_init1'].setEnabled(False),
                     ),
                   lambda w,obj:(
                      pm.StopGripper(),
                      stop_cmd('gripper' if not is_geh60 else 'gripper_geh'),
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
        'text':('Record FV','Stop recording'),
        #'enabled':False,
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:(
                      pm.fv.CallSrv('start_record'),
                     ),
                   lambda w,obj:(
                      pm.fv.CallSrv('stop_record'),
                     ) )}),
    'btn_align_windows': (
      'button',{
        'text': 'Align Windows',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: AlignWindows(config['WINDOW_ALIGNMENT']), }),
    }
  widgets_modbus= {
    'combo_modbus_c_robot': (
      'combobox',{
        'options':('Yaskawa','Fanuc'),
        'index':{'Yaskawa':0,'Fanuc':1}[ui_config['MODBUS_C_ROBOT']],
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum'),
        'ontextchanged': lambda w,obj:UpdateUIConfig('MODBUS_C_ROBOT',str(obj.currentText())) }),
    'lineedit_modbus_c_srv': (
      'combobox',{
        'options':('10.10.6.204','192.168.1.100','192.168.250.81'),
        'index': None,
        'editable': True,
        'text': ui_config['MODBUS_C_SRV'],
        'font_size_range': (8,24),
        'size_policy': ('expanding', 'minimum'),
        #'ontextchanged': lambda w,obj:UpdateUIConfig('MODBUS_C_SRV',str(obj.text())),
        'ontextchanged': lambda w,obj:UpdateUIConfig('MODBUS_C_SRV',str(obj.currentText())) }),
    'btn_modbus_client': (
      'buttonchk',{
        'text':('Modbus Client','Stop Modbus Cli'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      MapUIConfigToModbusConfig(),
                      run_cmd('modbus_client'),
                      w.widgets['btn_modbus_server'].setEnabled(False),
                     ),
                   lambda w,obj:(
                      stop_cmd('modbus_client'),
                      w.widgets['btn_modbus_server'].setEnabled(True),
                     ) )}),
    'btn_modbus_server': (
      'buttonchk',{
        'text':('Modbus Server','Stop Modbus Srv'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      MapUIConfigToModbusConfig(),
                      run_cmd('modbus_port_fwd'),
                      run_cmd('modbus_server'),
                      w.widgets['btn_modbus_client'].setEnabled(False),
                    ),
                   lambda w,obj:(
                      stop_cmd('modbus_server'),
                      w.widgets['btn_modbus_client'].setEnabled(True),
                     ) )}),
    }
  layout_init= (
    'boxh',None,(
      'label_init',
      ('boxv',None, (
        ('boxh',None,('label_config','combo_config_file_list')),
        ('boxh',None,('btn_init1', 'btn_init2')),
        ('boxh',None, ('btn_modbus_server',
                       ('boxv',None, (
                         ('boxh',None, ('combo_modbus_c_robot', 'lineedit_modbus_c_srv')),
                         'btn_modbus_client',
                         )),
                       ) if with_modbus else ()),
        ))
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
        'text': 'Gripper: ',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')}),
    'joy_grip': (
      'virtual_joystick',{
        'kind':'hbox',
        'stick_color':[255,128,128],
        'size_policy': ('expanding','expanding'),
        'onstickmoved': lambda w,obj:set_joy('grip',obj.position(),is_active=1), }),
    'lineedit_g_trg': (
      'lineedit',{
        'text': '',
        'validator': 'float',
        'font_size_range': (8,24),
        'size_policy': ('expanding', 'minimum')}),
    'btn_go_g_trg': (
      'button',{
        'text': 'Go',
        'font_size_range': (8,24),
        'size_policy': ('expanding', 'minimum'),
        'onclick': lambda w,obj:(pm.pub['set_target_pos'].publish(std_msgs.msg.Float64(float(w.widgets['lineedit_g_trg'].text())))
                                 if w.widgets['lineedit_g_trg'].text()!='' else None,), }),
    }

  layout_joy= (
    'boxv',None, (
      #('boxh',None,('label_grip','joy_grip','btn_grip_open')),
      ('boxh',None,(
        'label_grip',
        ('boxv',None, (
          ('boxh',None,('joy_grip',)),
          ('boxh',None,('lineedit_g_trg','btn_go_g_trg')),
          )),
        'btn_grip_open',
        )),
      ('boxh',None, ('btn_grasp','btn_hold','btn_openif','btn_stop')),
      ))

  widgets_fv_sensor= {
    'btn_show_fv': (
      'buttonchk',{
        'text':('Hide FV Windows','Show FV Windows'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      pm.fv.CallSrv('hide_windows'),
                     ),
                   lambda w,obj:(
                      pm.fv.CallSrv('show_windows'),
                     ) )}),
    'label_set_dim': (
      'label',{
        'text': 'Window Brightness: ',
        'font_size_range': (12,14),
        'size_policy': ('minimum', 'minimum')}),
    'combobox_set_dim_blob': (
      'combobox',{
        'options':map(lambda v:'blob:'+str(v),[0.0,0.3,0.7,1.0]),
        'index': 3,
        'font_size_range': (8,24),
        'size_adjust_policy': 'all_contents',
        'ontextchanged': lambda w,obj:pm.fv.CallSrv('set_dim_level','BlobTracker',obj.currentIndex()) }),
    'combobox_set_dim_pxv': (
      'combobox',{
        'options':map(lambda v:'pxv:'+str(v),[0.0,0.3,0.7,1.0]),
        'index': 1,
        'font_size_range': (8,24),
        'size_adjust_policy': 'all_contents',
        'ontextchanged': lambda w,obj:pm.fv.CallSrv('set_dim_level','ObjDetTracker',obj.currentIndex()) }),
    'label_calib': (
      'label',{
        'text': 'Calibration:      ',
        'font_size_range': (12,14),
        'size_policy': ('minimum', 'minimum')}),
    'btn_calib_r_blob': (
      'button',{
        'text':'r_blob',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvR('req_calibrate','BlobTracker',0),  }),
    'btn_calib_l_blob': (
      'button',{
        'text':'l_blob',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvL('req_calibrate','BlobTracker',0),  }),
    'btn_calib_r_pxv': (
      'button',{
        'text':'r_pxv',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvR('req_calibrate','ObjDetTracker',0),  }),
    'btn_calib_l_pxv': (
      'button',{
        'text':'l_pxv',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvL('req_calibrate','ObjDetTracker',0),  }),
    'label_save_calib': (
      'label',{
        'text': 'Save calibration: ',
        'font_size_range': (12,14),
        'size_policy': ('minimum', 'minimum')}),
    'btn_save_calib_r_blob': (
      'button',{
        'text':'r_blob',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvR('save_calibration','BlobTracker',0),  }),
    'btn_save_calib_l_blob': (
      'button',{
        'text':'l_blob',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvL('save_calibration','BlobTracker',0),  }),
    'btn_save_calib_r_pxv': (
      'button',{
        'text':'r_pxv',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvR('save_calibration','ObjDetTracker',0),  }),
    'btn_save_calib_l_pxv': (
      'button',{
        'text':'l_pxv',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvL('save_calibration','ObjDetTracker',0),  }),
    'label_load_calib': (
      'label',{
        'text': 'Load calibration: ',
        'font_size_range': (12,14),
        'size_policy': ('minimum', 'minimum')}),
    'btn_load_calib_r_blob': (
      'button',{
        'text':'r_blob',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvR('load_calibration','BlobTracker',0),  }),
    'btn_load_calib_l_blob': (
      'button',{
        'text':'l_blob',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvL('load_calibration','BlobTracker',0),  }),
    'btn_load_calib_r_pxv': (
      'button',{
        'text':'r_pxv',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvR('load_calibration','ObjDetTracker',0),  }),
    'btn_load_calib_l_pxv': (
      'button',{
        'text':'l_pxv',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvL('load_calibration','ObjDetTracker',0),  }),
    'label_set_trackbar_mode': (
      'label',{
        'text': 'Trackbar mode: ',
        'font_size_range': (12,14),
        'size_policy': ('minimum', 'minimum')}),
    'combobox_set_trackbar_blob': (
      'combobox',{
        'options':map(lambda v:'blob:'+str(v),range(6)),
        'index': 0,
        'font_size_range': (8,24),
        'size_adjust_policy': 'all_contents',
        'ontextchanged': lambda w,obj:pm.fv.CallSrv('set_trackbar_mode','BlobTracker',obj.currentIndex()) }),
    'combobox_set_trackbar_pxv': (
      'combobox',{
        'options':map(lambda v:'pxv:'+str(v),range(7)),
        'index': 0,
        'font_size_range': (8,24),
        'size_adjust_policy': 'all_contents',
        'ontextchanged': lambda w,obj:pm.fv.CallSrv('set_trackbar_mode','ObjDetTracker',obj.currentIndex()) }),
    'label_save_parameters': (
      'label',{
        'text': 'Save parameters: ',
        'font_size_range': (12,14),
        'size_policy': ('minimum', 'minimum')}),
    'btn_save_parameters_r': (
      'button',{
        'text':'right',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvR('save_parameters',config['FV_R_GUI_CONFIG']),  }),
    'btn_save_parameters_l': (
      'button',{
        'text':'left',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvL('save_parameters',config['FV_L_GUI_CONFIG']),  }),
    'label_obj_detection': (
      'label',{
        'text': 'Object detection(pxv): ',
        'font_size_range': (12,14),
        'size_policy': ('minimum', 'minimum')}),
    'btn_start_detect_obj': (
      'button',{
        'text':'Start',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrv('start_detect_obj'),  }),
    'btn_stop_detect_obj': (
      'button',{
        'text':'Stop',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrv('stop_detect_obj'),  }),
    'btn_clear_obj_l': (
      'button',{
        'text':'ClearObj(l)',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvL('clear_obj'),  }),
    'btn_clear_obj_r': (
      'button',{
        'text':'ClearObj(r)',
        'font_size_range': (8,24),
        'onclick': lambda w,obj:pm.fv.CallSrvR('clear_obj'),  }),
    }

  layout_fv_sensor= (
    'boxv',None, (
      'btn_show_fv',
      ('boxh',None, ('label_set_dim', ('boxv',None, (
                        ('boxh',None, ('combobox_set_dim_blob','combobox_set_dim_pxv',)),
                        ))
                     )),
      ('boxh',None, ('label_set_trackbar_mode', ('boxv',None, (
                        ('boxh',None, ('combobox_set_trackbar_blob','combobox_set_trackbar_pxv',)),
                        ))
                     )),
      ('boxh',None, ('label_save_parameters', ('boxv',None, (
                        ('boxh',None, ('btn_save_parameters_l','btn_save_parameters_r',)),
                        ))
                     )),
      ('boxh',None, ('label_obj_detection', ('boxv',None, (
                        ('boxh',None, ('btn_start_detect_obj','btn_stop_detect_obj','btn_clear_obj_l','btn_clear_obj_r',)),
                        ))
                     )),
      ('boxh',None, ('label_calib', ('boxv',None, (
                        ('boxh',None, ('btn_calib_l_blob','btn_calib_r_blob','btn_calib_l_pxv','btn_calib_r_pxv',)),
                        ))
                     )),
      ('boxh',None, ('label_save_calib', ('boxv',None, (
                        ('boxh',None, ('btn_save_calib_l_blob','btn_save_calib_r_blob','btn_save_calib_l_pxv','btn_save_calib_r_pxv',)),
                        ))
                     )),
      ('boxh',None, ('label_load_calib', ('boxv',None, (
                        ('boxh',None, ('btn_load_calib_l_blob','btn_load_calib_r_blob','btn_load_calib_l_pxv','btn_load_calib_r_pxv',)),
                        ))
                     )),
      ))

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

  widgets_ctrl_config= {
    }
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'min_gstep', (0.0,0.01,0.0001))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'min_obj_area', (0.0,1.0,0.01))
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
        ('boxh',None, CtrlConfigSliderLayout('min_obj_area') ),
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


  #def RunFVSignalPlot(w,obj):
    #plot_list= [p for p in config['PLOT_LIST'] if w.widgets['checkbox_{}'.format(p[1])].isChecked()]
    #if len(plot_list)==0:  return
    #cmd= cmds['fvsignal_plot'][0]+['--plots={}'.format(repr(plot_list).replace(' ',''))]
    #print('fvsignal_plot: {}'.format(cmd))
    #pm.RunBGProcess('fvsignal_plot',cmd)
  widgets_plot_cbs= {
    'checkbox_{}'.format(plot_label): (
        'checkbox',{
            'text': plot_label,
            'checked': enabled,
            'font_size_range': (8,24),
            'onclick': lambda w,obj,label=plot_label: UpdatePlotLoggerConfig(label, obj.isChecked()),
          })
      for (signal_name,plot_label,axis,index,enabled) in plot_logger_config}
  num_column= 3
  num_row= int(np.ceil(float(len(plot_logger_config))/num_column))
  layout_plot_cbs= (
    'boxh',None,(
      ('boxv',None,(
          'checkbox_{}'.format(plot_label)
          for (signal_name,plot_label,axis,index,enabled) in plot_logger_config[i:i+num_row]
        ))
        for i in range(0,len(plot_logger_config),num_row)
      ))
  widgets_plots= {
    'btn_plot': (
      'buttonchk',{
        'text':('Plot','Stop plot'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      #RunFVSignalPlot(w,obj),
                      run_cmd('fvsignal_plot'),
                     ),
                   lambda w,obj:(
                      stop_cmd('fvsignal_plot'),
                     ) )}),
    'btn_log': (
      'buttonchk',{
        'text':('Log','Stop log'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      run_cmd('fvsignal_log'),
                     ),
                   lambda w,obj:(
                      stop_cmd('fvsignal_log'),
                     ) )}),
    }
  layout_plots= (
    'boxv',None,(
      ('boxh',None, ('btn_plot','btn_log',)),
      ('boxv',None, (layout_plot_cbs,'spacer_cmn2')),
      ))

  widgets_debug= {
    'label_tools': (
      'label',{
        'text': 'Tools: ',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')}),
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
    'btn_ln_cams': (
      'button',{
        'text': 'Link Cameras',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: run_cmd('ln_cams') if AskYesNoDialog(w,'Link camera tool is executed in the terminal. Please see the terminal to proceed.\nDo you want to proceed?') else None }),
    'btn_gui_config_fv_l': (
      'buttonchk',{
        'text':('Camera Control (L)','Stop CamCon(L)'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      run_cmd('gui_config_fv_l'),
                     ),
                   lambda w,obj:(
                      stop_cmd('gui_config_fv_l'),
                     ) )}),
    'btn_gui_config_fv_r': (
      'buttonchk',{
        'text':('Camera Control (R)','Stop CamCon(R)'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      run_cmd('gui_config_fv_r'),
                     ),
                   lambda w,obj:(
                      stop_cmd('gui_config_fv_r'),
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
        'ontextchanged': lambda w,obj:None}),
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
    #'btn_dbg_exit': (
      #'button',{
        #'text': 'Exit',
        #'size_policy': ('expanding', 'fixed'),
        #'onclick': lambda w,obj: w.close(), }),
    'btn_dbg_relaunch': (
      'button',{
        'text': 'Relaunch',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: (setattr(pm,'flag_relaunch', True), w.close()), }),
    }
  if not is_geh60:
    widgets_debug_gripper= {
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
      }
  if is_geh60:
    widgets_debug_gripper= {
      'label_geh60': (
        'label',{
          'text': 'GEH6000IL: ',
          'font_size_range': (8,24),
          'size_policy': ('minimum', 'minimum')}),
      'btn_geh60_homing': (
        'button',{
          'text': 'Homing',
          'font_size_range': (8,24),
          #'size_policy': ('minimum', 'minimum'),
          'onclick': lambda w,obj: run_cmd('geh60_homing'), }),
      'btn_geh60_startmoveth': (
        'button',{
          'text': 'StartMoveTh',
          'font_size_range': (8,24),
          #'size_policy': ('minimum', 'minimum'),
          'onclick': lambda w,obj: run_cmd('geh60_startmoveth'), }),
      'btn_geh60_stopmoveth': (
        'button',{
          'text': 'StopMoveTh',
          'font_size_range': (8,24),
          #'size_policy': ('minimum', 'minimum'),
          'onclick': lambda w,obj: run_cmd('geh60_stopmoveth'), }),
      'btn_geh60_deactivate': (
        'button',{
          'text': 'Deactivate',
          'font_size_range': (8,24),
          #'size_policy': ('minimum', 'minimum'),
          'onclick': lambda w,obj: run_cmd('geh60_deactivate'), }),
      'btn_geh60_activate': (
        'button',{
          'text': 'Activate',
          'font_size_range': (8,24),
          #'size_policy': ('minimum', 'minimum'),
          'onclick': lambda w,obj: run_cmd('geh60_activate'), }),
      }
  if not is_geh60:
    layout_debug_gripper= (
      'boxh',None, (
        'label_dxlg',
        ('boxv',None, (
            ('boxh',None, ('btn_dxlg_reboot','btn_dxlg_factory_reset')),
            ))
      ))
  if is_geh60:
    layout_debug_gripper= (
      'boxh',None, (
        'label_geh60',
        ('boxv',None, (
            ('boxh',None, ('btn_geh60_homing','btn_geh60_deactivate','btn_geh60_activate')),
            ('boxh',None, ('btn_geh60_startmoveth','btn_geh60_stopmoveth')),
            ))
      ))
  layout_debug= (
    'boxv',None,(
      ('boxh',None, ('label_tools', ('boxv',None, (
                        ('boxh',None, ('btn_rviz','btn_ln_cams')),
                        ('boxh',None, ('btn_gui_config_fv_l','btn_gui_config_fv_r')),
                        )),
                     )),
      layout_debug_gripper if not sensor_app else ('boxh',None, ()),
      ('boxh',None, ('label_processes', ('boxv',None, (
                        'combobox_procs',
                        ('boxh',None, ('btn_update_proc_list','btn_terminate_proc','btn_kill_proc')),
                        #('boxh',None, ('btn_dbg_exit',)),
                        ('boxh',None, ('btn_dbg_relaunch',)),
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
          ('Main',('tab',None,(
              ('Operation',('boxv',None,(
                layout_init,
                ('boxh',None, ('btn_fv_record','btn_align_windows',)),
                layout_joy if not sensor_app else 'spacer_cmn1',
                ))),
              ('Signal',layout_plots),
            ))),
          ('Configuration',('tab',None,(
              ('FV_sensor',layout_fv_sensor),
              ('Control/1',layout_ctrl_config1),
              ('Control/2',layout_ctrl_config2),
            ))) if not sensor_app
          else ('Configuration',('tab',None,(
                  ('FV_sensor',layout_fv_sensor),
                ))),
          #('Initialize',layout_init),
          #('Joy',layout_joy),
          ('Advanced',layout_debug),
          )),
        'btn_exit',
        'spacer_cmn1')),
      ))

  app= InitPanelApp()
  win_size= (1000,500)
  if fullscreen:  #NOTE: fullscreen mode will work only with Qt5.
    print('Screen size: {}'.format(app.screens()[0].size()))
    screen_size= app.screens()[0].size()
    win_size= (screen_size.width(),screen_size.height())
  panel= TSimplePanel('Robot Operation Panel', size=win_size, font_height_scale=350.0)
  panel.AddWidgets(widgets_common)
  panel.AddWidgets(widgets_init)
  if not sensor_app:
    panel.AddWidgets(widgets_joy)
  panel.AddWidgets(widgets_fv_sensor)
  if not sensor_app:
    panel.AddWidgets(widgets_ctrl_config)
  panel.AddWidgets(widgets_plot_cbs)
  panel.AddWidgets(widgets_plots)
  panel.AddWidgets(widgets_debug)
  if not sensor_app:
    panel.AddWidgets(widgets_debug_gripper)
  if with_modbus:
    panel.AddWidgets(widgets_modbus)
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

  RunPanelApp(exit_at_close=False)

  if pm.flag_relaunch:
    RelaunchProgram()
