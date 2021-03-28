from mrbus import mrbeeSimple
import logging

mrbs = mrbeeSimple('/dev/ttyUSB1', 0xF0, logger=logging)

addr = 0x0f23

print("Clearing memory of 0xFF...")

mrbs.sendpkt(0x33, [ord('W'), addr&0xFF, addr>>8, 255, 255, 255, 255, 255, 255, 255, 255] )
mrbs.sendpkt(0x33, [ord('W'), (addr+8)&0xFF, addr>>8, 255, 255, 255, 255, 255, 255, 255, 255] )
mrbs.sendpkt(0x33, [ord('R'), addr&0xFF, addr>>8, 8] )
mrbs.sendpkt(0x33, [ord('R'), (addr+8)&0xFF, addr>>8, 8] )

#print("Single byte write/read...")

#mrbs.sendpkt(0x33, [ord('W'), addr&0xFF, 0xAA] )
#mrbs.sendpkt(0x33, [ord('R'), addr&0xFF] )

#mrbs.sendpkt(0x33, [ord('W'), addr&0xFF, 0x55] )
#mrbs.sendpkt(0x33, [ord('R'), addr&0xFF] )

mrbs.sendpkt(0x33, [ord('W'), addr&0xFF, addr>>8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11] )

mrbs.sendpkt(0x33, [ord('R'), addr&0xFF, addr>>8, 8] )
mrbs.sendpkt(0x33, [ord('R'), (addr+8)&0xFF, addr>>8, 8] )

while 1:
  p = mrbs.getpkt()
  if p!=None:
    if p.cmd!=ord('S'):
      print(p)
