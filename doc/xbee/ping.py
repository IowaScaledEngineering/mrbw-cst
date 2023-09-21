import serial
import time
import argparse
import sys

parser = argparse.ArgumentParser()
parser.add_argument('port')
parser.add_argument('baud')
args = parser.parse_args()

try:
	ser = serial.Serial(args.port, args.baud, timeout=0.1)
	ser.close()
	ser.open()
	time.sleep(1)
	ser.write(b'+++')
	time.sleep(1)
	read_val = ser.read(size=64).rstrip()
	sys.stdout.write(read_val.decode() + "" + "\n")

except serial.SerialException:
	print("Error opening serial port")
