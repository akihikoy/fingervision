#!/usr/bin/python
#\file    test1.py
#\brief   Test the FingerVision library for Python.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.28, 2022
# Usage:
#   $ ./test1.py fvp_1.yaml
#   $ ./test1.py ../standalone/file1.yaml
import fv
import sys
from rate_adjust3 import TRateAdjuster

if __name__=='__main__':
  config_file= sys.argv[1] if len(sys.argv)>1 else ""
  fv.StartThreads(config=config_file)

  rate= TRateAdjuster(30)
  while not fv.IsShutdown():
    fv.DisplayImages()
    rate.Sleep()

  fv.StopThreads()
