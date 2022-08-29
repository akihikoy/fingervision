#!/usr/bin/python

import sys, termios
import time
#import atexit
from select import select

#Detecting a keyboard hit without interrupting
#cf: http://code.activestate.com/recipes/572182-how-to-implement-kbhit-on-linux/
class TKBHit:
  def __init__(self,activate=True):
    self.is_curses_term= False
    self.termios= termios  #Ensure to use termios in __del__
    self.ask_timeout= 6000  #Timeout in Ask* functions.

    if activate:
      self.Activate()

  def __del__(self):
    self.Deactivate()

  def __enter__(self, *args, **kwargs):
    return self

  def __exit__(self, *args, **kwargs):
    self.Deactivate()

  def IsActive(self):
    return self.is_curses_term

  #Activate a new terminal for kbhit
  def Activate(self):
    # save the terminal settings
    self.fd= sys.stdin.fileno()
    self.new_term= termios.tcgetattr(self.fd)
    self.old_term= termios.tcgetattr(self.fd)

    # new terminal setting unbuffered
    self.new_term[3]= (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)

    #atexit.register(self.SetNormalTerm)
    self.SetCursesTerm()

  #Deactivate the terminal
  def Deactivate(self):
    self.SetNormalTerm()

  #Switch to normal terminal
  def SetNormalTerm(self):
    if self.is_curses_term:
      self.termios.tcsetattr(self.fd, self.termios.TCSAFLUSH, self.old_term)
      del self.fd
      del self.new_term
      del self.old_term
      self.is_curses_term= False

  #Switch to unbuffered terminal
  def SetCursesTerm(self):
    if self.is_curses_term:
      self.SetNormalTerm()
    termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)
    self.is_curses_term= True

  #Print a character
  def PutCh(self,ch):
    sys.stdout.write(ch)

  #Get a character
  def GetCh(self):
    return sys.stdin.read(1)

  #Get a character with echoing it
  def GetChE(self):
    ch= self.GetCh()
    self.PutCh(ch)
    return ch

  #Check a keyboard hit
  def CheckKBHit(self, timeout=0):
    dr,dw,de= select([sys.stdin], [], [], timeout)
    #print 'dr:',dr
    #print 'dw:',dw
    #print 'de:',de
    return dr <> []

  #Get a keyboard hit
  def KBHit(self, echo=False, timeout=0):
    if self.CheckKBHit(timeout):
      if echo:  return self.GetChE()
      else:  return self.GetCh()
      ##Try to clear buffer: (it does not work)
      #while self.CheckKBHit(0):
        #c= self.GetCh()
        #sys.stdout.write('>> %s\n'%ch)
        #sys.stdout.flush()
    return None

  #KBHit compatible AskYesNo.
  def AskYesNo(self):
    if self.IsActive():
      while 1:
        sys.stdout.write('  (y|n) > ')
        sys.stdout.flush()
        if self.CheckKBHit(self.ask_timeout):
          ans= self.GetChE()
          sys.stdout.write('\n')
          if ans=='y' or ans=='Y':  return True
          elif ans=='n' or ans=='N':  return False
    else:
      return AskYesNo()

  #KBHit compatible AskGen.
  #Usage: AskGen('y','n','c')
  def AskGen(self,*argv):
    assert(len(argv)>0)
    if self.IsActive():
      while 1:
        sys.stdout.write('  (%s) > ' % '|'.join(argv))
        sys.stdout.flush()
        if self.CheckKBHit(self.ask_timeout):
          ans= self.GetChE()
          sys.stdout.write('\n')
          for a in argv:
            if ans==a:  return a
    else:
      return AskGen(*argv)

#Wait for a key input (immediately quit after a key input).
def KBHitOnce():
  sys.stdout.flush()
  with TKBHit() as kbhit:
    while True:
      if kbhit.IsActive():
        key= kbhit.KBHit()
        if key is not None:
          return key
      else:  break
      time.sleep(0.005)
  return None

#KBHit version of AskYesNo (does not wait for pressing return key).
def KBHAskYesNo():
  sys.stdout.flush()
  with TKBHit() as kbhit:
    return kbhit.AskYesNo()
  return None

#KBHit version of AskGen (does not wait for pressing return key).
def KBHAskGen(*argv):
  sys.stdout.flush()
  with TKBHit() as kbhit:
    return kbhit.AskGen(*argv)
  return None


if __name__ == '__main__':
  kbhit= TKBHit()

  import time
  disp= '.'
  while 1:
    c=kbhit.KBHit()
    if c is not None:
      sys.stdout.write('> %r\n'%c)
      sys.stdout.flush()
      if c=='q':  break
      else:  disp= c
      #while kbhit(): getch()  #Clear buffer
    #else:
      #sys.stdout.write('no kbhit\n')
      #while kbhit():
        #ch= getch()  #Get the last one of buffer
        #sys.stdout.write('>>>> %s\n'%ch)
        #sys.stdout.flush()

    for i in range(40):
      sys.stdout.write(disp)
      sys.stdout.flush()
      time.sleep(0.05)
    sys.stdout.write('\n')

  print 'done'

