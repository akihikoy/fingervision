#!/usr/bin/python
#\file    test_qt1.py
#\brief   Example of fv_py with Qt.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.26, 2023
#   $ ./test_qt1.py fvp_1.yaml
#   $ ./test_qt1.py ../standalone/file1.yaml

import fv
import sys
from rate_adjust3 import TRateAdjuster
import multiprocessing as mp
import Queue
from PyQt4 import QtCore, QtGui

def FVLoop(config_file, queue_cmd, queue_out):
  fv.StartThreads(config=config_file)

  rate= TRateAdjuster(30)
  while not fv.IsShutdown():
    try:
      cmd= queue_cmd.get(block=False)
      if cmd=='quit':
        fv.SetShutdown()
        break
      elif cmd=='hide_win':
        fv.HideWindows()
      elif cmd=='show_win':
        fv.ShowWindows()
      elif cmd=='calib_0':
        name= fv.GetDisplayImageList()[0]
        print 'Calibrating {}...'.format(name)
        fv.SetCalibrationRequest(name)
      elif cmd=='calib_1':
        name= fv.GetDisplayImageList()[1]
        print 'Calibrating {}...'.format(name)
        fv.SetCalibrationRequest(name)
    except Queue.Empty:
      pass
    fv.DisplayImages()
    rate.Sleep()

  fv.StopThreads()
  queue_out.put('done')


class TButton(QtGui.QWidget):
  def __init__(self, queue_cmd, queue_out):
    QtGui.QWidget.__init__(self)
    self.queue_cmd= queue_cmd
    self.queue_out= queue_out
    self.InitUI()

  def InitUI(self):
    # Set window size.
    self.resize(120, 220)

    # Set window title
    self.setWindowTitle("fv_py with Qt")

    # Add a button
    btn1= QtGui.QPushButton('Hide', self)
    btn1.setCheckable(True)
    btn1.setFocusPolicy(QtCore.Qt.NoFocus)
    btn1.toggled.connect(lambda checked,btn1=btn1: (self.queue_cmd.put('hide_win'), btn1.setText('Show')) if checked
                                              else (self.queue_cmd.put('show_win'), btn1.setText('Hide')) )
    btn1.move(20, 20)

    btn2= QtGui.QPushButton('Exit', self)
    btn2.resize(btn2.sizeHint())
    btn2.clicked.connect(lambda b: (self.queue_cmd.put('quit'),
                                    self.queue_out.get(block=True),
                                    self.close()))
    btn2.move(20, 70)

    btn3= QtGui.QPushButton('Calib(0)', self)
    btn3.resize(btn3.sizeHint())
    btn3.clicked.connect(lambda b: self.queue_cmd.put('calib_0'))
    btn3.move(20, 120)

    btn4= QtGui.QPushButton('Calib(1)', self)
    btn4.resize(btn4.sizeHint())
    btn4.clicked.connect(lambda b: self.queue_cmd.put('calib_1'))
    btn4.move(20, 170)

    # Show window
    self.show()


if __name__=='__main__':
  config_file= sys.argv[1] if len(sys.argv)>1 else ""
  queue_cmd= mp.Queue()
  queue_out= mp.Queue()
  new_proc= mp.Process(target=FVLoop, args=(config_file, queue_cmd, queue_out))
  new_proc.start()

  # Create an PyQT4 application object.
  a= QtGui.QApplication(sys.argv)

  # The QWidget widget is the base class of all user interface objects in PyQt4.
  w= TButton(queue_cmd, queue_out)

  sys.exit(a.exec_())
