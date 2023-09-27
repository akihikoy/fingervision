#!/usr/bin/python
#\file    test_qt2.py
#\brief   Example of fv_py with Qt (the images are rendered with Qt).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.27, 2023
#   $ ./test_qt2.py fvp_1.yaml
#   $ ./test_qt2.py ../standalone/file1.yaml

import fv
import sys
from rate_adjust3 import TRateAdjuster
import multiprocessing as mp
import Queue
from PyQt4 import QtCore, QtGui
import cv2

class TImgDialog(QtGui.QDialog):
  def __init__(self, title, parent=None):
    super(TImgDialog, self).__init__(parent)
    self.setWindowTitle(title)
    #self.resize(321, 241)
    self.title= title
    self.cv_img= None
    self.q_img= None
    self.show()

  def setImage(self, frame):
    self.cv_img= frame
    height, width, byte_value = self.cv_img.shape
    byte_value= byte_value * width
    cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2RGB, self.cv_img)
    self.q_img= QtGui.QImage(self.cv_img, width, height, byte_value, QtGui.QImage.Format_RGB888)
    #print 'set image: {}: {}'.format(self.title, self.cv_img.shape)
    self.update()

  def paintEvent(self, event):
    #print 'paint request: {}: {}'.format(self.title, self.cv_img.shape)
    if self.cv_img is not None and self.q_img is not None:
      self.resize(self.cv_img.shape[1], self.cv_img.shape[0])
      painter= QtGui.QPainter()
      painter.begin(self)
      painter.drawImage(0, 0, self.q_img)
      painter.end()

  def keyPressEvent(self, event):
    super(TImgDialog, self).keyPressEvent(event)
    key= event.text()
    if key==' ':
      self.update()
    elif key=='q':
      self.close()

def FVLoop(config_file, queue_cmd, queue_out):
  fv.StartThreads(config=config_file, windows_hidden=True)

  img_windows= {}
  def create_windows():
    image_list= fv.GetDisplayImageList()
    for name in image_list:
      if name not in img_windows:
        img_windows[name]= TImgDialog(name)

  def destroy_windows():
    image_list= fv.GetDisplayImageList()
    for name in image_list:
      if name in img_windows:
        img_windows[name].close()
        del img_windows[name]

  def show_windows():
    image_list= fv.GetDisplayImageList()
    for name in image_list:
      if name in img_windows:
        img= fv.GetDispImage(name)
        #print 'Got display image: {}: {}'.format(name, img.shape)
        img_windows[name].setImage(img)

  def command_handler():
    if not fv.IsShutdown():
      try:
        cmd= queue_cmd.get(block=False)
        if cmd=='quit':
          destroy_windows()
          fv.SetShutdown()
          return
        elif cmd=='hide_win':
          destroy_windows()
        elif cmd=='show_win':
          create_windows()
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
      show_windows()

    else:
      fv.StopThreads()
      QtGui.QApplication.instance().quit()
      queue_out.put('done')

  app= QtGui.QApplication(sys.argv)
  app.setQuitOnLastWindowClosed(False)

  timer= QtCore.QTimer()
  timer.timeout.connect(command_handler)
  timer.start(1000/60)

  create_windows()
  app.exec_()
  print 'terminated'

  fv.StopThreads()
  queue_out.put('done')


class TFVApp(QtGui.QWidget):
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
  w= TFVApp(queue_cmd, queue_out)

  sys.exit(a.exec_())
