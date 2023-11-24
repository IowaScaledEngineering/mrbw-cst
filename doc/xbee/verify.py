import serial
import time
import argparse
import sys

def xbeeParam(param, expectedValue):
	ser.write(bytes(param + ',', 'utf-8'))
	read_val = ser.read(size=64).rstrip().decode()
	sys.stdout.write(param + "=" + read_val)
	if(expectedValue == int(read_val,16)):   # ID requires hex, use hex for all to make things simple
		sys.stdout.write(u"\u001b[32m OK\u001b[0m\n")
	else:
		sys.stdout.write(u"\u001b[31m Fail!\u001b[0m\n")

parser = argparse.ArgumentParser()
parser.add_argument('port')
parser.add_argument('baud')
args = parser.parse_args()

try:
	ser = serial.Serial(args.port, args.baud, timeout=0.1)
	ser.close()
	ser.open()
	sys.stdout.write("Entering command mode... ")
	time.sleep(1)
	ser.write(b'+++')
	time.sleep(1)
	read_val = ser.read(size=64).rstrip()
	sys.stdout.write(read_val.decode() + "" + "\n")

	print("Reading...")
	ser.write(b'AT')

	xbeeParam("ID", 0x225)
	xbeeParam("RR", 6)
	xbeeParam("RN", 3)
	xbeeParam("PL", 4)
	xbeeParam("PP", 8)
	xbeeParam("SM", 1)
	xbeeParam("BD", 7)
	xbeeParam("AP", 2)

	ser.write(b'CN\n')

except serial.SerialException:
	print("Error opening serial port")

