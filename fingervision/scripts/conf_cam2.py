#!/usr/bin/python
#\file    conf_cam2.py
#\brief   Camera configuration tool ver.2.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.29, 2021
'''
Usage:
  $ ./conf_cam2.py /dev/video0 "yaml:{CAMERA_CONFIG_IN_YAML}"
    CAMERA_CONFIG_IN_YAML: Camera configuration text in YAML format.
  $ ./conf_cam2.py /dev/video0 "b64:CAMERA_CONFIG_IN_BASE64"
    CAMERA_CONFIG_IN_BASE64: Camera configuration text in Base64 format.
  $ ./conf_cam2.py /dev/video0 "file:[KEYS:]CAMERA_CONFIG_YAML_FILE"
    CAMERA_CONFIG_YAML_FILE: Camera configuration YAML file name.
    KEYS: Keys to specify CAMERA_CONFIG in the YAML file.

  Examples:
    $ ./conf_cam2.py /dev/video0 "yaml:{Backlight Compensation: 0, Brightness: 0, Contrast: 64, Exposure (Absolute): 525, 'Exposure, Auto': 0, Gain: 0, Gamma: 45, Hue: 0, 'Iris, Absolute': 5, Saturation: 100, Sharpness: 20, White Balance Temperature: 5000, 'White Balance Temperature, Auto': 1}"
    $ ./conf_cam2.py /dev/video0 "b64:eJx1jr0KwjAURnef4m5VyJDWtkK2tog6V3C+lmCDbVKSGxDEdzep1c3t/nzn4zxr7O6DuvUEjRknqR2SMloAZ1DbeNfSuXltjCaLjgSUOYP9YzLOWwnr6urM4EluBBRZwVYAyffJoPJkkhk/oNLLMI4oIC8YHL2cT8nJKhfCS1MAdgxaJG8Xm5TzWNz2aKePUBawS69IQo0D6k7CWQZ/G6FQui157P2b+Jmlr9Ub+RpRhw=="
    $ ./conf_cam2.py /dev/video0 "file:cam_conf1.yaml"
    $ ./conf_cam2.py /dev/video0 "file:CameraParams:0:cam_conf4.yaml"

'''
import os,sys
import subprocess
import yaml
import base64
import zlib
import re

def ExecCmd(cmd):
  p= subprocess.Popen(cmd, stdout=subprocess.PIPE)
  (stdout, stderr)= p.communicate()
  exit_code= p.wait()
  return stdout.strip() if stdout is not None else stdout, stderr.strip() if stderr is not None else stderr, exit_code

def EncodeDictB64(d, with_compress=True):
  d_yaml= yaml.dump(d)
  if with_compress:  d_yaml_c= zlib.compress(d_yaml)
  else:  d_yaml_c= d_yaml
  d_b64= base64.b64encode(d_yaml_c)
  return d_b64

def DecodeDictB64(d_b64, with_compress=True):
  d_yaml_c= base64.b64decode(d_b64)
  if with_compress:  d_yaml= zlib.decompress(d_yaml_c)
  else:  d_yaml= d_yaml_c
  d= yaml.load(d_yaml)
  return d

def SetCtrlValue(cam_dev, ctrl, value):
  #NOTE: '--' is neccessary for nevative values (not neccessary for positive ones).
  stdout,stderr,ec= ExecCmd(['uvcdynctrl', '-d', os.path.realpath(cam_dev), '-s', ctrl, '--', str(value)])

def SetCtrlValues(cam_dev, ctrl_values):
  for ctrl,value in ctrl_values.iteritems():
    SetCtrlValue(cam_dev, ctrl, value)

def LoadFromYAML(filename):
  assert(os.path.exists(filename))
  fp= open(filename,'r')
  pos1= fp.tell()
  line1= fp.readline()
  if line1[:6]!='%YAML:':
    fp.seek(pos1)
  Loader= yaml.Loader
  yaml.add_multi_constructor('!', lambda loader,tag_suffix,node: loader.construct_yaml_map(node), Loader=Loader)
  yaml.add_multi_constructor('tag', lambda loader,tag_suffix,node: loader.construct_yaml_map(node), Loader=Loader)
  return yaml.load(fp.read(), Loader=Loader)

if __name__=='__main__':
  if len(sys.argv)<=1:  raise Exception('Specify a camera device.')
  if len(sys.argv)<=2:  raise Exception('Specify camera configuration.')
  cam_dev= sys.argv[1] if len(sys.argv)>1 else '/dev/video0'
  values= sys.argv[2]

  if values[:5]=='yaml:':
    d= yaml.load(values[5:])
  elif values[:4]=='b64:':
    d= DecodeDictB64(values[4:])
  elif values[:5]=='file:':
    keys_filename= values[5:].split(':')
    filename= keys_filename[-1]
    keys= [int(key) if re.match(r'\+?[0-9]+',key) else key for key in keys_filename[:-1]]
    #d= yaml.load(open(filename,'r').read())
    d= LoadFromYAML(filename)
    for key in keys:  d= d[key]
  else:
    raise Exception('Unrecognized value type:', values)
  SetCtrlValues(cam_dev, d)
