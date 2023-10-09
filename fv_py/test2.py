#!/usr/bin/python
#\file    test2.py
#\brief   Test the FingerVision library for Python.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.30, 2022
# Usage:
#   $ ./test2.py fvp_1.yaml
#   $ ./test2.py ../standalone/file1.yaml
#   The program is started without windows.
#   On the terminal:
#     Press h to switch show/hide the windows.
#     Press q to quit.
import fv
import sys
from rate_adjust3 import TRateAdjuster
from kbhit2 import TKBHit

if __name__=='__main__':
  config_file= sys.argv[1] if len(sys.argv)>1 else ""
  windows_hidden= True
  fv.StartThreads(config=config_file, windows_hidden=windows_hidden)

  disp_images= fv.GetDisplayImageList()
  print 'disp_images:',disp_images
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
      windows_hidden= not fv.HandleWindowVisibilityRequest()
      if not windows_hidden:
        for name in disp_images:
          fv.DisplayImage(name)
        if not fv.HandleKeyEvent():  fv.SetShutdown()
      #if not fv.DisplayImages():  fv.SetShutdown()
      rate.Sleep()

  fv.StopThreads()
