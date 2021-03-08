# *************************************************************************
# Title:    Interface Driver for MRBus wired or wireless connections
# Authors:  Mark Finn <mark@mfinn.net>
#           Michael D. Petersen <railfan@drgw.net>
#           Nathan D. Holmes <maverick@drgw.net>
# File:     mrbus.py
# License:  GNU General Public License v3
# 
# LICENSE:
#   Copyright (C) 2018 Mark Finn, Michael Petersen & Nathan Holmes
#     
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 3 of the License, or
#   any later version.
# 
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
# 
# DESCRIPTION:
#   This class provides a way to interface with a MRBus control network
#   ( https://www.iascaled.com/blog/an-introduction-to-mrbus/ ), either via 
#   the Iowa Scaled MRB-CI2 to a wired network or via an XBee to a wireless
#   network connected by serial (likely an FTDI adapter or similar) to the 
#   host computer.
# 
# *************************************************************************

import serial
import time
from collections import deque
import sys

class packet(object):
  def __init__(self, dest, src, cmd, data):
    self.dest=dest
    self.src=src
    self.cmd=cmd
    self.data=data

  def __hash__(self):
    return hash(repr(self))

  def __eq__(self, other):
    return repr(self)==repr(other)

  def __repr__(self):
    return "mrbus.packet(0x%02x, 0x%02x, 0x%02x, %s)"%(self.dest, self.src, self.cmd, repr(self.data))

  def __str__(self):
    c='(0x%02X'%self.cmd
    if self.cmd >= 32 and self.cmd <= 127:
      c+=" '%c')"%self.cmd
    else:
      c+="    )"
    return "packet(0x%02X->0x%02X) %s %2d:%s"%(self.src, self.dest, c, len(self.data), ["0x%02X"%d for d in self.data])

class node(object):
  def __init__(self, mrb, addr):
    def handler(p):
      if p.src==self.addr and (p.dest==mrb.addr or p.dest==0xff):
        self.pkts.append(p)
      return True #eat packet

    self.mrb=mrb
    self.addr=addr
    self.pkts=deque()
    self.hint=mrb.install(handler, -1)


  def __dell__(self):
    self.mrb.remove(self.hint)

  def __str__(self):
    return "node(%02) %s"%(self.addr)

  def sendpkt(self, data):
    self.mrb.sendpkt(self.addr, data)

  def getpkt(self, timeout=None):
    if timeout==None:
      while len(self.pkts) == 0:
        self.mrb.pump(timeout=None)
    else:
      start = time.time()
      now = start
      while now - start <= timeout and len(self.pkts)==0:
        self.mrb.pump(timeout=start + timeout - now)
        now = time.time()

    if len(self.pkts) == 0:
      return None
    return self.pkts.popleft()



class mrbusSimple(object):
  def __init__(self, port, addr, logger=None, extra=False):

    if type(port)==str:
      port = serial.Serial(port, 115200, timeout=.1, rtscts=True)

    self.serial = port
    self.rxBuffer = bytearray()
    time.sleep(.1)
    while port.inWaiting():
      port.read(port.inWaiting())
    port.write(b':CMD NS=00;\r')
    if extra:
      port.write(b':CMD MM=00;\r')
    else:
      port.write(b':CMD MM=01;\r')
  
    port.timeout=0

    self.pktlst=[]

    self.logger = logger
    if logger is None:
      self.logger = logging.getLogger('mrbus')
    
    self.logger.info("Instantiated mrbusSimple from %s" % (port.name))

    self.addr=addr
#    self.buf=deque()

  def disconnect(self):
     self.serial.close()

#needs timeout functionality
#  def readline(self)
#    while not self.linebuf():
#      r=self.serial.read(max(1, self.serial.inWaiting()))
#      while '\n' in r:
#        i = r.index('\n')
#        self.linebuf.append(list(self.linecbuf)+r[:i+1]        
#        self.linecbuf=deque()
#        r=r[i+1:]
#      if r:
#        self.linecbuf.extend(r)
#    return self.linebuf.leftpop()


# This is ugly, but now works in Python3
# Probably needs to be revisited and fixed up
  def getpkt(self):

    while True:
      incoming = self.serial.read()
      if incoming is None or len(incoming) == 0:
        break
       
      incomingBytes = bytearray(incoming).strip(b'\r')
      self.rxBuffer = self.rxBuffer + incomingBytes
       
    if b'\n' in self.rxBuffer:
      # We've got complete lines, do something with them
      eolIdx = self.rxBuffer.index(b'\n')
      cmdStr = str(self.rxBuffer[0:eolIdx].decode('utf-8')).strip()
      self.rxBuffer = self.rxBuffer[eolIdx+1:]
      #print("Found command [%s]" % cmdStr)

      if cmdStr == "Ok":
        return None
      
      if len(cmdStr)<2 or cmdStr[0:2] != 'P:':
        self.logger.error('E3<<<'+cmdStr)
        return None

      d=[int(v,16) for v in cmdStr[2:].split()]
      if len(d)<6 or len(d)!=d[2]:
        self.logger.error('E4<<<'+cmdStr)
        return None
      
      return packet(d[0], d[1], d[5], d[6:])

    return None

  def sendpkt(self, dest, data, src=None):
    if src == None:
      src = self.addr
    s = ":%02X->%02X"%(src, dest)
    for d in data:
      if type(d) == str:
        d=ord(d)
      s+=" %02X"%(d&0xff)
    s+=";\r"
    self.logger.debug('>>>MRBUS ' +s)
    self.serial.write(s)


class mrbeeSimple(object):
  def __init__(self, port, addr, logger=None, extra=False):

    self.rxBuffer = []
    self.rxExcapeNext = 0
    self.rxProcessing = 0
    self.leds = { 'D6':False, 'D7':False, 'D8':False, 'D9':False }

    if type(port)==str:
      port = serial.Serial(port, 115200, timeout=.1, rtscts=True, stopbits=serial.STOPBITS_TWO)

    self.serial = port

    time.sleep(.1)
    while port.inWaiting():
      port.read(port.inWaiting())
 
    port.timeout=0

    self.pktlst=[]

    self.logger = logger
    if logger is None:
      self.logger = logging.getLogger('mrbus')

    self.logger.info("Instantiated mrbeeSimple from %s" % port.name)

    self.addr=addr
   
    self.setLED('D6', False)
    self.setLED('D7', False)
    self.setLED('D8', False)
    self.setLED('D9', False)

  def disconnect(self):
    try:  # These very well might fail if the port went away
       self.setLED('D6', False)
       self.setLED('D7', False)
       self.setLED('D8', False)
       self.setLED('D9', False)  
       time.sleep(0.1)
    except:
       pass
    self.serial.close()

  def getpkt(self):

    while True:
       incomingStr = self.serial.read()
       if incomingStr is None or len(incomingStr) == 0:
          break
       
       #incomingByte = ord(incomingStr[0])
       incomingByte = 0xFF & incomingStr[0]
       if 0x7E == incomingByte:
#          self.log(0, "mrbee starting new packet")
          self.rxBuffer = [ incomingByte ]
          self.rxExcapeNext = 0
          self.rxProcessing = 1 
          self.rxExpectedPktLen = 255
          continue
          
       elif 0x7D == incomingByte:
#          self.log(0, "mrbee setting escape")
          self.rxExcapeNext = 1
          continue
       
       else:
          if self.rxProcessing == 0:
#             self.log(0, "mrbee got byte %02X outside processing, ignoring\n" % incomingByte)
             continue
       
          if self.rxExcapeNext != 0:
             incomingByte = incomingByte ^ 0x20
             self.rxExcapeNext = 0

#          self.log(0, "mrbee got byte %02X\n" % incomingByte)       
          
          self.rxBuffer.append(incomingByte)
    
          if len(self.rxBuffer) == 3:
             self.rxExpectedPktLen = self.rxBuffer[1] * 256 + self.rxBuffer[2] + 4

          if len(self.rxBuffer) == self.rxExpectedPktLen:
#             self.log(0, "mrbee - think we may have a packet")                 
             pktChecksum = 0
             for i in range(3, self.rxExpectedPktLen):
                pktChecksum = (pktChecksum + self.rxBuffer[i]) & 0xFF
                               
             if 0xFF != pktChecksum:
                # Error, conintue
                self.logger.error("mrbee - checksum error - checksum is %02X" % (pktChecksum))
                continue      

             if 0x80 == self.rxBuffer[3]:
                # 16 bit addressing
                pktDataOffset = 14
             elif 0x81 == self.rxBuffer[3]:
                # 64 bit addressing
                pktDataOffset = 8
             else:
                # Some other API frame, just dump it
                self.rxBuffer = [ ]
                self.rxExcapeNext = 0
                self.rxProcessing = 0

#                for i in range(0, self.rxExpectedPktLen-1):
#                   print "Byte %02d: 0x%02X" % (i-pktDataOffset, self.rxBuffer[i])
             
             return packet(self.rxBuffer[pktDataOffset + 0], self.rxBuffer[pktDataOffset + 1], self.rxBuffer[pktDataOffset + 5], self.rxBuffer[(pktDataOffset + 6):])

    return None

  def getLED(self, ledRefdes):
     if ledRefdes not in self.leds:
        return False
     return self.leds[ledRefdes]

  def setLED(self, ledRefdes, ledState):
#     if ledState:
#        state = "On"
#     else:
#        state = "Off"
#     self.log(0, "mrbee - setting XBee LED %d to %s" % (ledNum, state))

     pins = {'D6':2, 'D7':3, 'D8':1, 'D9':0}

     if ledRefdes not in pins:
       return
     if type(ledState) is not bool:
       return
     
     self.leds[ledRefdes] = ledState
     
     txBuffer = [ ]
     txBuffer.append(0x7E)          # 0 - Start 
     txBuffer.append(0x00)          # 1 - Len MSB
     txBuffer.append(0x05)          # 2 - Len LSB - five bytes
     txBuffer.append(0x08)          # 3 - API being called - AT CMD
     txBuffer.append(0x00)          # 4 - Frame identifier
     txBuffer.append(0x44)          # 5 - 'D'
     txBuffer.append(0x30 + pins[ledRefdes]) # 6 - '0'-'4'
     if ledState is True:
        txBuffer.append(5)       # 8 - '5' for on
     else:
        txBuffer.append(4)       # 8 - '4' for off
     
     xbeeChecksum = 0
     for i in range(3, len(txBuffer)):
        xbeeChecksum = (xbeeChecksum + txBuffer[i]) & 0xFF
     xbeeChecksum = (0xFF - xbeeChecksum) & 0xFF;
     txBuffer.append(xbeeChecksum)     

     txBufferEscaped = [ txBuffer[0] ]
     escapedChars = frozenset([0x7E, 0x7D, 0x11, 0x13])

     for i in range(1, len(txBuffer)):
        if txBuffer[i] in escapedChars:
           txBufferEscaped.append(0x7D)
           txBufferEscaped.append(txBuffer[i] ^ 0x20)
        else:
           txBufferEscaped.append(txBuffer[i])

     self.serial.write(txBufferEscaped)  
     return   
  

  def sendpkt(self, dest, data, src=None):
     if src == None:
        src = self.addr

     txPktLen = 10 + len(data)   # 5 MRBus overhead, 5 XBee, and the data

     txBuffer = [ ]
     txBuffer.append(0x7E)       # 0 - Start 
     txBuffer.append(0x00)       # 1 - Len MSB
     txBuffer.append(txPktLen)   # 2 - Len LSB
     txBuffer.append(0x01)       # 3 - API being called - transmit by 16 bit address
     txBuffer.append(0x00)       # 4 - Frame identifier
     txBuffer.append(0xFF)       # 5 - MSB of dest address - broadcast 0xFFFF
     txBuffer.append(0xFF)       # 6 - LSB of dest address - broadcast 0xFFFF
     txBuffer.append(0x00)       # 7 - Transmit Options
     
     txBuffer.append(dest)           # 8 / 0 - Destination
     txBuffer.append(src)            # 9 / 1 - Source
     txBuffer.append(len(data) + 5)  # 10/ 2 - Length
     txBuffer.append(0)              # 11/ 3 - CRC High
     txBuffer.append(0)              # 12/ 4 - CRC Low

#     print "MRBee transmitting from %02X to %02X" % (src, dest)

     for b in data:
        txBuffer.append(int(b) & 0xFF)

     crc = mrbusCRC16Calculate(txBuffer[8:])
     txBuffer[11] = 0xFF & crc
     txBuffer[12] = 0xFF & (crc >> 8)

     xbeeChecksum = 0
     for i in range(3, len(txBuffer)):
        xbeeChecksum = (xbeeChecksum + txBuffer[i]) & 0xFF
     xbeeChecksum = (0xFF - xbeeChecksum) & 0xFF;
     txBuffer.append(xbeeChecksum)

     txBufferEscaped = [ txBuffer[0] ]
     
     escapedChars = frozenset([0x7E, 0x7D, 0x11, 0x13])

     for i in range(1, len(txBuffer)):
        if txBuffer[i] in escapedChars:
           txBufferEscaped.append(0x7D)
           txBufferEscaped.append(txBuffer[i] ^ 0x20)
        else:
           txBufferEscaped.append(txBuffer[i])

#     print "txBufferEscaped is %d bytes" % (len(txBufferEscaped))
#     pkt = ""
#     for i in txBufferEscaped:
#        pkt = pkt + "%02x " % (i)
#     print pkt
     
     self.serial.write(txBufferEscaped)     
     
def mrbusCRC16Calculate(data):
   mrbusPktLen = data[2]
   crc = 0
   
   for i in range(0, mrbusPktLen):
      if i == 3 or i == 4:
         continue
      else:
         a = data[i]
      crc = mrbusCRC16Update(crc, a)
      
   return crc


def mrbusCRC16Update(crc, a):
   MRBus_CRC16_HighTable = [ 0x00, 0xA0, 0xE0, 0x40, 0x60, 0xC0, 0x80, 0x20, 0xC0, 0x60, 0x20, 0x80, 0xA0, 0x00, 0x40, 0xE0 ]
   MRBus_CRC16_LowTable =  [ 0x00, 0x01, 0x03, 0x02, 0x07, 0x06, 0x04, 0x05, 0x0E, 0x0F, 0x0D, 0x0C, 0x09, 0x08, 0x0A, 0x0B ]
   crc16_h = (crc>>8) & 0xFF
   crc16_l = crc & 0xFF
   
   i = 0
   
   while i < 2:
      if i != 0:
         w = ((crc16_h << 4) & 0xF0) | ((crc16_h >> 4) & 0x0F)
         t = (w ^ a) & 0x0F
      else:
         t = (crc16_h ^ a) & 0xF0
         t = ((t << 4) & 0xF0) | ((t >> 4) & 0x0F)
         
      crc16_h = (crc16_h << 4) & 0xFF
      crc16_h = crc16_h | (crc16_l >> 4)
      crc16_l = (crc16_l << 4) & 0xFF
      
      crc16_h = crc16_h ^ MRBus_CRC16_HighTable[t]
      crc16_l = crc16_l ^ MRBus_CRC16_LowTable[t]
      
      i = i + 1
      
   return (crc16_h<<8) | crc16_l
      
class mrbus(object):
  def disconnect(self):
     self.mrbs.disconnect()

  def __init__(self, port, addr=None, logger=None, extra=False, busType='mrbus'):
    if type(port)==str:
      if busType == 'mrbus':
         port = serial.Serial(port, 115200, rtscts=True, timeout=.1)
      elif busType == 'mrbee':
         port = serial.Serial(port, 115200, rtscts=True, stopbits=serial.STOPBITS_TWO, timeout=.1)

    self.logger = logger
    if self.logger is None:
       self.logger = logging.getLogger("mrbus")
       self.logger.setLevel(logging.DEBUG)
     

    if busType == 'mrbus':
       self.mrbs = mrbusSimple(port, addr, logger, extra)
       self.busType = 'mrbus'
       
    elif busType == 'mrbee':
       self.mrbs = mrbeeSimple(port, addr, logger, extra)
       self.busType = 'mrbee'

    self.pktlst=[]
    self.handlern=0
    self.handlers=[]
    self.fakeLEDs={'D6':False, 'D7':False, 'D8':False, 'D9':False}

    self.mrbs.logger.info("mrbus object instantiated [%s] from [%s]" % (self.busType, port.name))

    #find an address to use
    if addr==None:
      self.mrbs.logger.info("finding address to use")
      for addr in xrange(254, 0, -1):
        found = self.testnode(addr, replyto=0xff)
        if not found:
          break
      if found:
        self.mrbs.logger.error("no available address found to use")
        raise Exception("no available address found to use")
     
    self.addr=addr
    self.mrbs.addr=addr

    self.mrbs.logger.info("using address %d" % (addr))

  def setXbeeLED(self, ledRefdes, ledState):
    if self.busType == 'mrbee':
      self.mrbs.setLED(ledRefdes, ledState)
    else:
      if ledState is bool:
        self.fakeLEDs[ledRefdes] = ledState

  def getXbeeLED(self, ledRefdes):
    if self.busType == 'mrbee':
      return self.mrbs.getLED(ledRefdes)
    else:
      if ledRefdes in self.fakeLEDs:
        return self.fakeLEDs[ledRefdes]
      else:
        return False

  def sendpkt(self, addr, data, src=None):
    self.mrbs.sendpkt(addr, data, src)

  def getpkt(self):
    return self.mrbs.getpkt()

  def getnode(self, dest):
    return node(self, dest)

  def install(self, handler, where=-1):
    #interpret index differently than list.insert().  -1 is at end, 0 is at front
    self.mrbs.logger.info("mrbus install handler")
    if where<0:
      if where == -1:
        where = len(self.handlers)
      else:
        where+=1

    hint=self.handlern
    self.handlern+=1
    self.handlers.insert(where, (hint, handler))

  def remove(self, hint):
    self.mrbs.logger.info("mrbus remove handler")
    self.handlers = [h for h in self.handlers if h[0]!=hint]


  def pump(self, timeout=None):
    done=False
    to = self.mrbs.serial.timeout
    if timeout != None:
      timeout=max(0,timeout)
    self.mrbs.serial.timeout=timeout
    while not done:
      p = self.getpkt()
      if p:
        self.mrbs.serial.timeout=0
        for hint,h in self.handlers:
          r = h(p)
          if r:
            break
      else:
        done=True
    self.mrbs.serial.timeout=to

  def testnode(self, addr, replyto=None, wait=2):
    found=False

    def pingback(p):
      if p.src==addr:
        found=True
      if p.cmd=='a':
        return True #eat pings
      return False

    if replyto == None:
      replyto = self.addr

    hint = self.install(pingback, 0)

    t=time.time()
    n=0
    while time.time()-t < wait and not found:
      x=(time.time()-t)/.2
      if x > n:
        self.sendpkt(addr, ['A'], src=replyto)
        n+=1
      tn=time.time()
      to=min(wait+t-tn, n*.2+t-tn)
      self.pump(to)

    self.remove(hint)
    return found


        
  def scannodes(self, pkttype=ord('A'), rettype=ord('a'), wait=2):
    targets=set()

    def pingback(p):
      if p.src!=self.mrbs.addr and p.src!=0 and p.src!=0xff and p.cmd==rettype:
        targets.add(p.src)
      return False

    hint = self.install(pingback, 0)

    t=time.time()
    n=0
    while time.time()-t < wait:
      x=(time.time()-t)/.3
      if x > n:
        self.sendpkt(0xff, [pkttype])
        n+=1
      tn=time.time()
      to=min(wait+t-tn, n*.3+t-tn)
      self.pump(to)

    self.remove(hint)
    return sorted(targets)



###mrbus example use:
def mrbus_ex(ser):
  mrb = mrbus(ser)
#  mrb = mrbus(ser, logall=True, logfile=sys.stderr)
  nodes = mrb.scannodes()
  print('nodes: '+', '.join(str(n) for n in nodes))


###node example use:
def node_ex(ser):
  mrb = mrbus(ser)
  nodes = mrb.scannodes()
  assert nodes

  n=mrb.getnode(nodes[0])

  n.sendpkt(['V'])
  p=n.getpkt(timeout=3)
  if p:
    print(p)
  else:
    print('no packet returned')


###mrbusSimple example use:
def mrbussimple_ex(ser):
  addr=0
  mrbs = mrbusSimple(ser, addr)
#  mrbs = mrbusSimple(ser, logall=True, logfile=sys.stderr)
  t=time.time()
  while time.time()-t < 3:
    mrbs.sendpkt(0xff, ['A'])
    time.sleep(.3)
    while 1:
      p = mrbs.getpkt()
      if p==None:
        break
      if p.src!=addr and p.src!=0 and p.src!=0xff:
        print('recieved reply from node:', p.src)


if __name__ == '__main__':
  with serial.Serial('/dev/ttyUSB0', 115200, timeout=0, rtscts=True) as ser:

#    mrbussimple_ex(ser)
#    mrbus_ex(ser)
    node_ex(ser)


