#!/usr/bin/python
import roslib; roslib.load_manifest('fv_gripper_ctrl')
import rospy
from ay_py.core import *
from ay_py.ros import *
import tf
import sensor_msgs.msg
import fv_sensor
import std_msgs.msg
import std_srvs.srv
import fingervision_msgs.msg
import fingervision_msgs.srv


def DecodeNamedVariableMsg(msg):
  accessor= {msg.INT:msg.data_i, msg.FLOAT:msg.data_f, msg.BOOL:msg.data_b, msg.STR:msg.data_s}
  if msg.type==msg.NONE:
    if msg.form==msg.SCALAR:  return msg.name, None
    elif msg.form==msg.LIST_1D:
      if not (len(msg.sizes)==1 and msg.sizes[0]==0):
        raise Exception('Invalid content (NONE/LIST_1D): {}'.format(msg))
      return msg.name, []
    else:
      raise Exception('Invalid content: {}'.format(msg))
  if msg.form==msg.SCALAR:
    return msg.name, accessor[msg.type][0]
  elif msg.form==msg.LIST_1D:
    data= accessor[msg.type]
    if not (len(msg.sizes)==1 and msg.sizes[0]==len(data)):
      raise Exception('Invalid content (LIST_1D): {}'.format(msg))
    return msg.name, data
  elif msg.form==msg.LIST_2D:
    data= accessor[msg.type]
    if not (len(msg.sizes)>=2 and np.prod(msg.sizes)==len(data)):
      raise Exception('Invalid content (LIST_2D): {}'.format(msg))
    data= np.array(data).reshape(msg.sizes)
    return msg.name, data

def DecodeNamedVariableListMsg(msg):
  rawdata= (msg.data.data if isinstance(msg,fingervision_msgs.msg.NamedVariableListStamped) else
            msg.data      if isinstance(msg,fingervision_msgs.msg.NamedVariableList)
                          else msg)
  data= [DecodeNamedVariableMsg(d) for d in rawdata]
  return {name:value for (name,value) in data}

def EncodeNamedVariableMsg(name, var):
  msg= fingervision_msgs.msg.NamedVariable()
  msg.name= name
  field= {msg.INT:'data_i', msg.FLOAT:'data_f', msg.BOOL:'data_b', msg.STR:'data_s'}
  assign= lambda type,v: setattr(msg,field[type], v)
  def detect_type(scalar):
    dtype= scalar.dtype
    if   np.issubdtype(dtype,np.dtype(str).type):  return msg.STR
    elif np.issubdtype(dtype,np.dtype(float).type):  return msg.FLOAT
    elif np.issubdtype(dtype,np.dtype(int).type):  return msg.INT
    elif np.issubdtype(dtype,np.dtype(bool).type):  return msg.BOOL
    raise Exception('Unrecognized variable: {} type: {}'.format(scalar,dtype))
  if isinstance(var,(list,np.ndarray)):
    if len(var)==0:
      msg.type= msg.NONE
      msg.form= msg.LIST_1D
      msg.sizes= [0]
    else:
      var= np.array(var)
      msg.type= detect_type(var)
      msg.form= msg.LIST_1D if len(var.shape)==1 else msg.LIST_2D
      msg.sizes= var.shape
      assign(msg.type, var.ravel().tolist())
  else:
    if var is None:
      msg.type= msg.NONE
      msg.form= msg.SCALAR
      msg.sizes= []
    else:
      msg.type= detect_type(np.array(var))
      msg.form= msg.SCALAR
      msg.sizes= []
      assign(msg.type, [var])
  return msg


def CreateGripperDriver(gripper_type, gripper_node='gripper_driver'):
  gripper= None
  param= None
  if gripper_type in ('RHP12RNGripper','RHP12RNAGripper'):
    mod= __import__('ay_py.ros.rbt_rhp12rn',globals(),None,('TRHP12RNGripper',))
    gripper= mod.TRHP12RNGripper(node_name=gripper_node)
    #gripper= TSimGripper2F1(('RHP12RNGripper','ThGripper'),pos_range=[0.0,0.109])
    gripper.Init()
    param={
      'lx': [0.0,0.0,0.218, 0.5,-0.5,0.5,0.5],
      'bound_box':{
        'dim': [0.16,0.04,0.218],
        'center': [0.0,0.0,0.109, 0.0,0.0,0.0,1.0],
        }
      }
  elif gripper_type=='DxlGripper':
    mod= __import__('ay_py.ros.rbt_dxlg',globals(),None,('TDxlGripper',))
    gripper= mod.TDxlGripper(node_name=gripper_node)
    #gripper= TSimGripper2F1((gripper_type,),pos_range=[0.0,0.095])
    gripper.Init()
    param={
      'lx': [0.0,0.0,0.16, 0.5,-0.5,0.5,0.5],
      'bound_box':{
        'dim': [0.08,0.02,0.16],
        'center': [0.0,0.0,0.08, 0.0,0.0,0.0,1.0],
        }
      }
  elif gripper_type=='EZGripper':
    mod= __import__('ay_py.ros.rbt_ezg',globals(),None,('TEZGripper',))
    gripper= mod.TEZGripper(node_name=gripper_node)
    #gripper= TSimGripper2F1((gripper_type,),pos_range=[0.0,0.150])
    gripper.Init()
    param={
      'lx': [0,0,0, 0.0,-0.70710678,0.0,0.70710678],
      'bound_box':{
        'dim': [0.25,0.20,0.045],
        'center': [-0.125,0.0,0.0, 0.0,0.0,0.0,1.0],
        }
      }
  elif gripper_type.startswith('DxlpO2Gripper'):
    finger_type= gripper_type.replace('DxlpO2Gripper_','')
    fts= {'Straight1':'st1','SRound1':'sr1','Fork1':'f1'}[finger_type]
    mod= __import__('ay_py.ros.rbt_dxlpo2',globals(),None,('TDxlpO2Gripper',))
    gripper= mod.TDxlpO2Gripper(node_name=gripper_node, finger_type=finger_type)
    #gripper= TSimGripper2F1((gripper_type,),pos_range={'st1':[0.0,0.300],'sr1':[0.0,0.1950],'f1':[-0.0189,0.200]}[fts])
    gripper.Init()
    param={
      'lx': [0.0,0.0,{'st1':0.31,'sr1':0.22,'f1':0.22}[fts], 0.5,-0.5,0.5,0.5],
      'bound_box':{
        'dim': [0.30,0.154,{'st1':0.31,'sr1':0.22,'f1':0.22}[fts]],
        'center': [0.0,0.0,{'st1':0.155,'sr1':0.11,'f1':0.11}[fts], 0.0,0.0,0.0,1.0],
        }
      }
  elif gripper_type=='DxlpY1Gripper':
    mod= __import__('ay_py.ros.rbt_dxlpy1',globals(),None,('TDxlpY1Gripper',))
    gripper= mod.TDxlpY1Gripper(node_name=gripper_node)
    #gripper= TSimGripper2F1((gripper_type,),pos_range=[0.0,0.133])
    gripper.Init()
    param={
      'lx': [0.0,0.0,0.264, 0.5,-0.5,0.5,0.5],
      'bound_box':{
        'dim': [0.115,0.120,0.264],
        'center': [0.0,0.0,0.132, 0.0,0.0,0.0,1.0],
        }
      }
  elif gripper_type.startswith('GEH60'):
    mod= __import__('ay_py.ros.rbt_geh6000il',globals(),None,('TGEH6000ILGripper',))
    gripper= mod.TGEH6000ILGripper(node_name=gripper_node)
    gripper.Init()
    pos_range= gripper.PosRange()
    param={
      'lx': [0.0,0.0,0.1655, 0.5,-0.5,0.5,0.5],
      'bound_box':{
        'dim': [pos_range[1]+0.01 if pos_range is not None else 0.13, 0.062, 0.1655],
        #WARNING: dim[y]=0.062 is inaccurate for GEH6040 because it ignores the i/f board.
        'center': [0.0,0.0,0.1655/2., 0.0,0.0,0.0,1.0],
        }
      }
  else:
    raise Exception('CreateGripperDriver: Unknown gripper type: {}'.format(gripper_type))
  return gripper, param


def JoyCallback(state, steps, wsteps, gsteps, data):
  gsteps[0]= 0.0
  steps[:]= [0.0]*3
  wsteps[:]= [0.0]*3
  state[1]= 'no_cmd'
  state[3]= False

  #multiplier= 1.0
  multiplier= (1.0+data.axes[5])*0.5 + (1.0-data.axes[2])*2.0  #RT, LT
  #if data.buttons[1] > 0:
    #multiplier= 2.0 * multiplier

  axes= [(ax if abs(ax)>0.15 else 0.0) for ax in data.axes]

  if data.buttons[7] > 0:  #START
    state[0]= 'quit'
    return

  if data.buttons[1] > 0:  #B
    state[1]= 'cmd_B'
    #return
  if data.buttons[2] > 0:  #X
    state[1]= 'cmd_X'
    #return

  if data.buttons[3] > 0:  #Y
    state[1]= 'cmd_Y'
    #return

  dpad_btn= [1 if btn>0 else 0 for btn in data.buttons[11:15]]
  if any(dpad_btn):
    if sum(dpad_btn)==1:  #Need to press exactly one d-pad button at the same time
      if data.buttons[11] > 0:  #LEFT
        state[1]= 'cmd_left'
        #return
      if data.buttons[12] > 0:  #RIGHT
        state[1]= 'cmd_right'
        #return
      if data.buttons[13] > 0:  #UP
        state[1]= 'cmd_up'
        #return
      if data.buttons[14] > 0:  #DOWN
        state[1]= 'cmd_down'
        #return

  if data.buttons[0] > 0:  #A
    state[1]= 'grip'
    gsteps[0]= multiplier * axes[0]
    #return

  if data.buttons[5] <= 0:  #not RB
    state[3]= False
  else:
    state[3]= True


class TFVGripper(TROSUtil):
  def __init__(self):
    super(TFVGripper,self).__init__()
    self.config_path= [os.path.join(os.environ['HOME'],subdir) for subdir in ['config/','data/config/',]]
    self.gripper= None  #Gripper driver.
    self.g_param= None  #Gripper parameter dict.
    self.fv= fv_sensor.TFVSensor()  #FV sensor utility.
    self.frame_id= 'base_link'
    self.fv_ctrl_param= TContainer()
    self.cnt= TContainer()  #Utility container for the scripts.
    self.script_is_active= False
    self.script_thread= None
    self.sensors= {}
    self.viz= None

    self.state_locker= threading.RLock()
    self.g_target= None

  def __del__(self):
    self.Cleanup()
    if TFVGripper is not None:  super(TFVGripper,self).__del__()
    print 'TFVGripper: done',self

  def Cleanup(self):
    self.StopScript()
    if self.gripper is not None:
      self.gripper.Cleanup()
      del self.gripper
      self.gripper= None
    if TFVGripper is not None:  super(TFVGripper,self).Cleanup()

  def Setup(self, gripper_type, gripper_node, fv_names, fv_nodes):
    self.gripper,self.g_param= CreateGripperDriver(gripper_type, gripper_node=gripper_node)
    self.fv.Setup(self.gripper, self.g_param, self.frame_id, fv_names=fv_names, node_names=fv_nodes)
    self.viz= TSimpleVisualizerArray(rospy.Duration(1.0), name_space='fvgripper', frame=self.frame_id)
    self.LoadCtrlParams()

    self.AddPub('gripper_pos','~gripper_pos',std_msgs.msg.Float64)
    self.AddPub('target_pos','~target_pos',std_msgs.msg.Float64)
    self.AddPub('active_script','~active_script',std_msgs.msg.String)
    self.AddPub('fvsignals','~fvsignals',fingervision_msgs.msg.NamedVariableListStamped)

    self.AddSub('set_target_pos', '~set_target_pos', std_msgs.msg.Float64, lambda msg:self.SetGripperTarget(msg.data))
    self.AddSrv('run_script', '~run_script', fingervision_msgs.srv.SetString,
                lambda req:(self.RunScript(req.data),
                            fingervision_msgs.srv.SetStringResponse())[-1])
    self.AddSrv('stop_script', '~stop_script', std_srvs.srv.Empty,
                lambda req:(self.StopScript(), std_srvs.srv.EmptyResponse())[-1])

  def VizGripper(self):
    xw= [0,0,0, 0,0,0,1]
    viz= self.viz
    mid= 0
    mid= viz.AddCoord(xw, scale=[0.03,0.002], alpha=1.0, mid=mid)
    lw_xe= self.g_param['lx']
    bb_dim= self.g_param['bound_box']['dim']
    bb_center= self.g_param['bound_box']['center']
    mid= viz.AddCube(Transform(xw,bb_center), bb_dim, rgb=viz.ICol(3), alpha=0.5, mid=mid)
    #Visualize finger pads:
    gpos= self.gripper.Position()
    if gpos is not None:
      lw_xgl= Transform(lw_xe,[0,+0.5*gpos,0, 0,0,0,1])
      lw_xgr= Transform(lw_xe,[0,-0.5*gpos,0, 0,0,0,1])
      mid= viz.AddCube(Transform(xw,lw_xgl), [0.015,0.003,0.03], rgb=viz.ICol(1), alpha=0.8, mid=mid)
      mid= viz.AddCube(Transform(xw,lw_xgr), [0.015,0.003,0.03], rgb=viz.ICol(1), alpha=0.8, mid=mid)
      mid= viz.AddCoord(Transform(xw,lw_xe), scale=[0.01,0.001], alpha=1.0, mid=mid)
    viz.Publish()

  def LoadScript(self, script_name):
    try:
      #mod= __import__(script_name,globals(),None,(script_name,))
      mod= SmartImportReload(script_name)
      return mod
    except ImportError:
      print 'No script named: {}'.format(script_name)
    return None

  def GetSensorFunctions(self, sensor_name):
    # mod= __import__(sensor_name,globals(),None,(sensor_name,))
    mod= self.LoadScript(sensor_name)
    if mod is None:  return None,None
    f_reset,f_get= getattr(mod,'Reset',None), getattr(mod,'Get',None)
    return f_reset,f_get

  #Load sensor scripts specified by sensor_name_list.
  #  run_reset: Control the timing to run Reset function:
  #    'all': All sensors, 'only_new': Newly added sensors, 'none': No sensors.
  def LoadAllSensors(self, sensor_name_list, run_reset='only_new'):
    #Remove sensors that are not included in sensor_name_list:
    remove_list= [s for s in self.sensors.iterkeys() if s not in sensor_name_list]
    for s in remove_list:  del self.sensors[s]
    #Load sensors and store them in self.sensors:
    for sensor_name in sensor_name_list:
      print 'Loading sensor {}'.format(sensor_name)
      f_reset,f_get= self.GetSensorFunctions(sensor_name)
      is_new= False
      if sensor_name not in self.sensors:
        self.sensors[sensor_name]= dict()
        is_new= True
      self.sensors[sensor_name]['f_reset']= f_reset
      self.sensors[sensor_name]['f_get']= f_get
      if run_reset=='all' or (is_new and run_reset=='only_new'):
        try:
          f_reset(self)
        except Exception as e:
          print '  Error in executing {}.Reset: {}'.format(sensor_name, e)
          del self.sensors[sensor_name]

  #Run Get functions of all loaded sensors for a given fv_data (snapshot of self.fv.data)
  # and return time_stamp and {sensor_name:value} dict.
  def GetAllSensorValues(self, fv_data=None):
    if fv_data is None:  fv_data= copy.deepcopy(self.fv.data)
    sensor_values= {}
    time_stamps= [tm for tm in fv_data.tm_last_topic if tm is not None]
    time_stamp= max(time_stamps) if len(time_stamps)>0 else None
    for sensor_name,d in self.sensors.iteritems():
      try:
        sensor_values[sensor_name]= d['f_get'](self, fv_data)
      except Exception as e:
        print 'Sensor {} error: {}'.format(sensor_name, e)
    return time_stamp, sensor_values

  def CopySensorValuesToMsg(self, time_stamp, sensor_values):
    msg= fingervision_msgs.msg.NamedVariableListStamped()
    msg.header.stamp= time_stamp
    msg.data.data= [EncodeNamedVariableMsg(sensor_name, sensor_values[sensor_name] if sensor_name in sensor_values else None)
                    for sensor_name,d in self.sensors.iteritems()]
    return msg

  def GetScriptFunctions(self, script_name):
    # mod= __import__(script_name,globals(),None,(script_name,))
    mod= self.LoadScript(script_name)
    if mod is None:  return None,None
    f_run,f_loop= getattr(mod,'Run',None), getattr(mod,'Loop',None)
    if f_run is None and f_loop is None:
      print 'In script {}, both Run and Loop are not defined'.format(script_name)
      return None,None
    if f_run is not None and f_loop is not None:
      print 'In script {}, both Run and Loop are defined'.format(script_name)
      return None,None
    return f_run,f_loop

  def RunScript(self, script_name, with_update_params=True):
    self.StopScript()
    if with_update_params:  self.LoadCtrlParams()  #TODO:FIXME:This is tentative.
    f_run,f_loop= self.GetScriptFunctions(script_name)
    if f_run is not None:
      self.script_is_active= True
      CPrint(2,'{} is called'.format(script_name))
      try:
        f_run(self)
      except Exception as e:
        print 'Script {} error in Run: {}'.format(script_name, e)
      self.script_is_active= False
    elif f_loop is not None:
      self.script_is_active= True
      self.script_thread= threading.Thread(name=script_name,
                                           target=lambda:self.ScriptLoopExecutor(f_loop))
      CPrint(2,'{} is started'.format(script_name))
      self.script_thread.start()

  def ScriptLoopExecutor(self, f_loop):
    th= self.script_thread
    script_name= th.name if th is not None else '???'
    try:
      f_loop(self)
    except Exception as e:
      print 'Script {} error in Loop: {}'.format(script_name, e)
    CPrint(2,'{} is stopped'.format(script_name))
    self.script_is_active= False
    self.script_thread= None

  def StopScript(self):
    th= self.script_thread
    self.script_is_active= False
    if th is not None:  th.join()
    self.script_thread= None

  def IsScriptActive(self):
    return self.script_is_active

  def ActiveScript(self):
    return None if self.script_thread is None else self.script_thread.name

  def GripperPosition(self):
    return self.gripper.Position()

  def GripperMoveTo(self, pos=None, max_effort=100.0, speed=100.0, blocking=False):
    if pos is None:
      pos= self.GripperTarget()
    else:
      self.SetGripperTarget(pos)
    if pos is not None:
      self.gripper.Move(pos, max_effort=max_effort, speed=speed, blocking=blocking)

  def GripperTarget(self):
    with self.state_locker:
      return self.g_target

  def SetGripperTarget(self, g_target):
    with self.state_locker:
      self.g_target= g_target

  def CtrlLoop(self):
    #TODO:FIXME:Put in the param list.
    sensor_name_list= [
      'fv.area_l',
      'fv.area_r',
      'fv.area',
      'fv.center_l',
      'fv.center_r',
      'fv.d_center_l',
      'fv.d_center_r',
      'fv.da_area',
      'fv.da_center_norm',
      'fv.da_orientation',
      'fv.force_l',
      'fv.force_r',
      'fv.is_detected_l',
      'fv.is_detected_r',
      'fv.num_force_change',
      'fv.orientation_l',
      'fv.orientation_r',
      'fv.slip_l',
      'fv.slip_r',
      'fv.slip',
      ]
    self.LoadAllSensors(sensor_name_list)

    if self.gripper.Is('DxlGripper'):
      active_holding= [False]

    steps= [0.0, 0.0, 0.0]
    wsteps= [0.0, 0.0, 0.0]
    gsteps= [0.0]
    state= ['run', 'no_cmd', None, False]  #run/quit, no_cmd/CMD, ARM, ACTIVE_BTN

    ctrl_freq= 200.0  #TODO:Put in the param list.
    rate_adjuster= rospy.Rate(ctrl_freq)

    g_range= self.gripper.PosRange()
    while not self.gripper.IsInitialized() and not rospy.is_shutdown():
      rospy.sleep(0.05)
    self.SetGripperTarget(self.GripperPosition())
    self.GripperMoveTo()

    self.AddSub('joy', 'joy', sensor_msgs.msg.Joy, lambda msg: JoyCallback(state, steps, wsteps, gsteps, msg))

    try:
      with TKBHit() as kbhit:
        while state[0]=='run' and not rospy.is_shutdown():
          if kbhit.IsActive():
            key= kbhit.KBHit()
            if key=='q':
              break;
            elif key is not None:
              state[1]= 'key_'+str(key)
          else:
            break

          if state[1]=='key_i' or state[1]=='cmd_Y':
            if self.ActiveScript()=='fv.inhand':
              self.StopScript()
            else:
              self.RunScript('fv.inhand')
            state[1]= 'no_cmd'
          elif state[1]=='cmd_left':
            self.RunScript('fv.open')
            state[1]= 'no_cmd'
          elif state[1]=='cmd_right':
            if state[3]:
              self.RunScript('fv.grasp')
            else:
              self.StopScript()
            state[1]= 'no_cmd'
          elif state[1]=='cmd_up':
            if state[3]:
              self.RunScript('fv.hold')
            else:
              self.StopScript()
            state[1]= 'no_cmd'
          elif state[1]=='cmd_down':
            if state[3]:
              self.RunScript('fv.openif')
            else:
              self.StopScript()
            state[1]= 'no_cmd'

          elif state[1]=='key_[':
            gsteps[0]= 0.005
            state[1]= 'grip'
          elif state[1]=='key_]':
            gsteps[0]= -0.005
            state[1]= 'grip'
          elif state[1]=='key_{':
            gsteps[0]= 0.01
            state[1]= 'grip'
          elif state[1]=='key_}':
            gsteps[0]= -0.01
            state[1]= 'grip'

          if state[1]=='grip':
            if self.gripper.Is('DxlGripper'):
              if not active_holding[0]:
                self.gripper.StartHolding()
                active_holding[0]= True
            if abs(gsteps[0])>1.0e-5:
              g_target= self.GripperPosition() + 1.0/ctrl_freq*gsteps[0]
              if g_target<g_range[0]:  g_target= g_range[0]
              if g_target>g_range[1]:  g_target= g_range[1]
              self.SetGripperTarget(g_target)
            else:
              state[1]= 'no_cmd'

          if not self.script_is_active:
            self.GripperMoveTo()

          if not state[1]=='grip':
            if self.gripper.Is('DxlGripper'):
              if active_holding[0]:
                self.gripper.StopHolding()
                active_holding[0]= False

          time_stamp,sensor_values= self.GetAllSensorValues()
          fvsignals_msg= self.CopySensorValuesToMsg(time_stamp,sensor_values)

          self.pub.gripper_pos.publish(std_msgs.msg.Float64(self.GripperPosition()))
          self.pub.target_pos.publish(std_msgs.msg.Float64(self.GripperTarget()))
          self.pub.active_script.publish(std_msgs.msg.String(self.ActiveScript()))
          self.pub.fvsignals.publish(fvsignals_msg)

          self.VizGripper()
          rate_adjuster.sleep()

    finally:
      self.DelSub('joy')
      if self.gripper.Is('DxlGripper'):
        if active_holding[0]:
          self.gripper.StopHolding()
          active_holding[0]= False
      print 'Finished'

  def LoadCtrlParams(self):
    #Common control parameters:
    self.fv_ctrl_param.min_gstep= 0.0005
    self.fv_ctrl_param.effort= 100.0

    #Load default control parameters of scripts.
    curr_dir= os.path.dirname(__file__)
    for script in sorted(os.listdir(os.path.join(curr_dir,'fv'))):
      if not script.endswith('.py'):  continue
      mod= self.LoadScript('fv.{}'.format(script[:-3]))
      f_set= getattr(mod,'SetDefaultParams',None)
      #print 'debug',script,'fv.{}'.format(script[:-3]),f_set
      if f_set is None:  continue
      f_set(self)

    #Load fv_ctrl parameters from files.
    CONFIG_FILE= 'fv_ctrl.yaml'
    ctrl_params= {}
    for dir_path in fvg.config_path:
      file_path= os.path.join(dir_path,CONFIG_FILE)
      if os.path.exists(file_path):
        InsertDict(ctrl_params, LoadYAML(file_path))
    for k,v in ctrl_params.iteritems():
      fvg.fv_ctrl_param[k]= v


if __name__ == '__main__':
  rospy.init_node('fv_gripper_ctrl')

  gripper_type= rospy.get_param('~gripper_type', 'RHP12RNGripper')
  gripper_node= rospy.get_param('~gripper_node', 'gripper_driver')
  fv_names= rospy.get_param('~fv_names', {RIGHT:'fvp_1_r',LEFT:'fvp_1_l'})
  fv_nodes= rospy.get_param('~fv_nodes', None)
  def convert_key(key):
    table= {'left':LEFT,'LEFT':LEFT,'Left':LEFT,
            'right':RIGHT,'RIGHT':RIGHT,'Right':RIGHT}
    if isinstance(key,str):
      if key in table:  return table[key]
      return int(key)
    return key
  print 'fv_names: {}'.format(fv_names)
  fv_names= {convert_key(key):value for key,value in fv_names.iteritems()}

  fvg= TFVGripper()
  fvg.Setup(gripper_type, gripper_node, fv_names, fv_nodes)

  fvg.CtrlLoop()
  fvg.Cleanup()

