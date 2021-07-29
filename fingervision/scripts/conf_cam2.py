#!/usr/bin/python
#\file    conf_cam2.py
#\brief   Camera configuration tool ver.2.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.29, 2021
import os,sys
import subprocess
import yaml
import base64
import zlib

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
  stdout,stderr,ec= ExecCmd(['uvcdynctrl', '-d', cam_dev, '-s', ctrl, '--', str(value)])

def SetCtrlValues(cam_dev, ctrl_values):
  for ctrl,value in ctrl_values.iteritems():
    SetCtrlValue(cam_dev, ctrl, value)

if __name__=='__main__':
  if len(sys.argv)<=1:  raise Exception('Specify a camera device.')
  if len(sys.argv)<=2:  raise Exception('Specify camera configuration.')
  cam_dev= sys.argv[1] if len(sys.argv)>1 else '/dev/video0'
  values= sys.argv[2]

  if values[:5]=='yaml:':
    d= yaml.load(values[5:])
  elif values[:4]=='b64:':
    d= DecodeDictB64(values[4:])
  else:
    raise Exception('Unrecognized value type:', values)
  SetCtrlValues(cam_dev, d)
