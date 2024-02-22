from mrbus import mrbeeSimple
import logging

mrbs = mrbeeSimple('/dev/ttyUSB1', 0xF0, logger=logging)

while 1:
  p = mrbs.getpkt()
  if p!=None:
    print(p)
