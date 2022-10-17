#!/usr/bin/python
#\file    test3.py
#\brief   Test the FingerVision library for Python.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.30, 2022
# Usage:
#   $ ./test3.py fvp_1.yaml
#   $ ./test3.py ../standalone/file1.yaml
#   The program is started without windows.
#   On the terminal:
#     Press h to switch show/hide the windows.
#     Press b to switch start/stop displaying the BlobMoves data.
#     Press p to switch start/stop displaying the ProxVision data.
#     Press c to do calibration with the first window.
#     Press C to do calibration with the second window.
#     Press q to quit.
import fv
import sys
from rate_adjust3 import TRateAdjuster
from kbhit2 import TKBHit

if __name__=='__main__':
  config_file= sys.argv[1] if len(sys.argv)>1 else ""
  windows_hidden= True
  fv.StartThreads(config=config_file, windows_hidden=windows_hidden)

  print 'GetDisplayImageList:',fv.GetDisplayImageList()
  print 'GetNumCameras:',fv.GetNumCameras()
  disp_blob_moves= False
  disp_prox_vision= False
  rate= TRateAdjuster(30)
  with TKBHit() as kbhit:
    while not fv.IsShutdown():
      c= kbhit.KBHit()
      if c=='q':
        fv.SetShutdown()
        break
      elif c=='h':
        windows_hidden= not windows_hidden
        if windows_hidden:  fv.HideWindows()
        else:  fv.ShowWindows()
      elif c=='b':
        disp_blob_moves= not disp_blob_moves
      elif c=='p':
        disp_prox_vision= not disp_prox_vision
      elif c in ('c','C'):
        name= fv.GetDisplayImageList()[{'c':0,'C':1}[c]]
        print 'Calibrating {}...'.format(name)
        fv.SetCalibrationRequest(name)
      if not windows_hidden:
        for name in fv.GetDisplayImageList():
          fv.DisplayImage(name)
        if not fv.HandleKeyEvent():  fv.SetShutdown()
      if disp_blob_moves:
        print fv.GetBlobMoves(0)
      if disp_prox_vision:
        print fv.GetProxVision(0)
      rate.Sleep()
  fv.StopThreads()
