# FV Gripper Extension Script

This directory contains scripts of the external functions for the FV Gripper control system.
Those external functions include sensor signal processing scripts and general purpose scripts.

The extension script framework makes adding extra signal processing and gripper control easy.

The signal processing scripts are executed in each step of the loop, and the results are sent to the unified ROS topic: `/fv_gripper_ctrl/fvsignals` (`fingervision_msgs/NamedVariableListStamped` type).

The general purpose scripts can be executed via ROS service: `/fv_gripper_ctrl/run_script` (`fingervision_msgs/SetString`).

In order to achieve this flexibility, the scripts need to follow the format.


# Extension Script Format


## Common Function Format

Each script should include following functions:

### Help

```
def Help():
```

- Parameters: `None`
- Returns:
  - message: `str` of the help message.

Function to return a usage (help) message.

### SetDefaultParams

```
def SetDefaultParams(fvg):
```

- Parameters:
  - `fvg`: Instance of `fv_gripper_ctrl` object.
- Returns: `None`

(OPTIONAL) Function to set default configuration parameters.
The configuration parameters can be stored in the `fvg.fv_ctrl_param` container.
`SetDefaultParams` is executed when the configuration parameters are loaded from a file (`fv_gripper_ctrl.LoadCtrlParams`).

Note that the `fvg.fv_ctrl_param` container is loaded from a configuration file of YAML format in the setup step of the `fv_gripper_ctrl` node, and can be saved as well.



## Signal Processing Script

Each signal processing script should include following functions:

### Reset

```
def Reset(fvg):
```

- Parameters:
  - `fvg`: Instance of `fv_gripper_ctrl` object.
- Returns: `None`

Function to reset the internal state for the signal processing scripts.
The internal state can be stored in the `fvg.cnt` container.
`Reset` is executed at the beginning of the control loop of `fv_gripper_ctrl`.

Note that the `fvg.cnt` container is not saved.

### Get

```
def Get(fvg, fv_data):
```

- Parameters:
  - `fvg`: Instance of `fv_gripper_ctrl` object.
  - `fv_data`: Snapshot of the raw FV signals at the current frame.
- Returns:
  - data value.

Function to get the signal processed value.
This function is the main calculation part of the signal processing script, which converts the raw FV signals to certain value.

The raw FV signals are also stored in `fvg.fv.data`, but do not use it as it is updated in the subscriber callback during computing the extension scripts.
Its snapshot (`copy.deepcopy`) is stored into `fv_data` at the beginning of each control loop.

The return data value should be able to store into `fingervision_msgs/NamedVariable`.

- SCALAR: `None, int, float, bool, str`
- LIST_1D: `list` of SCALAR.
- LIST_2D: `list` of `list` of SCALAR (all sub lists must have the same length).



## General Purpose Script

Each general purpose script should include either `Run` or `Loop`.
DO NOT DEFINE BOTH.

- `Run`:
  - Script for one shot execution.
  - Executed within the `run_script` service callback.
  - Can be executed during executing a `Loop` type script.
  - Does not change `fv_gripper_ctrl.IsScriptActive()`.
  - Does not change `fv_gripper_ctrl.ActiveScript()`.
- `Loop`:
  - Script for loop type execution.
  - Executed as a thread function.
  - Only a single `Loop` type script can be executed at one time.  When `run_script` is called during executing one `Loop` type script, the previous one is stopped and the new one is started.
  - `fv_gripper_ctrl.IsScriptActive()` becomes `True` when the thread is active.
  - `fv_gripper_ctrl.ActiveScript()` (published via `/fv_gripper_ctrl/active_script` topic) returns the active script name.


### Run

```
def Run(fvg):
```

- Parameters:
  - `fvg`: Instance of `fv_gripper_ctrl` object.
- Returns: `None`

When the `run_script` service is called with the script name, the `Run` function is executed within the service callback.
Note that the `Run` function blocks the `run_script` service.

### Loop

```
def Loop(fvg):
```

- Parameters:
  - `fvg`: Instance of `fv_gripper_ctrl` object.
- Returns: `None`

When the `run_script` service is called with the script name, the `Loop` function is executed as a thread.
The `Loop` function should quit when the ROS node is going to shutdown, or the `fvg.script_is_active` flag becomes `False`.
The `Loop` function also can quit at arbitrary timing.

A template implementation is like:

```
def Loop(fvg):
  fv_data= fvg.fv.data  #Reference to access the current FV data.
  g_pos= fvg.GripperPosition()  #Get the current gripper position.
  while fvg.script_is_active and not rospy.is_shutdown():
    #Calculate g_pos with fv_data.
    fvg.GripperMoveTo(pos=g_pos, max_effort=fvg.fv_ctrl_param.effort, speed=1.0, blocking=True)
    rospy.sleep(0.04)
```


# ROS Services

Related to the extension scripts, following services are available with the ROS node `/fv_gripper_ctrl`:

- `/fv_gripper_ctrl/run_script` (`fingervision_msgs/SetString`)
    - Launch the script.
    - The selection of Run/Loop is automatic.
- `/fv_gripper_ctrl/stop_script` (`std_srvs/Empty`)
    - Stop the running Loop type script.
- `/fv_gripper_ctrl/reset_script` (`fingervision_msgs/SetString`)
    - Call the `Reset` function of the script (if defined).
- `/fv_gripper_ctrl/help_script` (`fingervision_msgs/SetGetString`)
    - Call the `Help` function of the script and respond the return (if defined).

