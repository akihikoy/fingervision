#!/usr/bin/python
#\file    rate_adjust3.py
#\brief   Rate adjuster 2.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.29, 2022
import time

class TRateAdjuster(object):
  def __init__(self, frequency):
    self.expected_next_start_= time.time()
    self.actual_prev_start_= time.time()
    self.expected_cycle_time_= (1.0 / frequency)
    self.actual_cycle_time_= 0.0
  def Sleep(self):
    expected_curr_start= self.expected_next_start_
    expected_next_end= expected_curr_start + self.expected_cycle_time_
    actual_prev_end= time.time()
    if actual_prev_end < expected_curr_start:  expected_next_end= actual_prev_end+self.expected_cycle_time_
    sleep_time= expected_next_end - actual_prev_end
    self.actual_cycle_time_= actual_prev_end-self.actual_prev_start_
    self.actual_prev_start_= actual_prev_end
    self.expected_next_start_= expected_next_end
    if sleep_time <= 0.0:
      if actual_prev_end > expected_next_end+self.expected_cycle_time_:  self.expected_next_start_= actual_prev_end
      return
    time.sleep(sleep_time)
  def Reset(self):
    self.expected_next_start_= time.time()
  def ActualCycleTime(self):
    return self.actual_cycle_time_
  def ExpectedCycleTime(self):
    return self.expected_cycle_time_

if __name__=='__main__':
  import random
  rate= TRateAdjuster(2)
  t0= time.time()
  while True:
    print '---------',time.time()-t0
    t0= time.time()
    s= 0
    for i in xrange(int(600*random.random())):
      s+= i
      time.sleep(0.0001)
    print i
    rate.Sleep()
    #time.sleep(0.5)
