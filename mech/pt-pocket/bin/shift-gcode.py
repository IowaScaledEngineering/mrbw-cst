#!/usr/bin/python

from cStringIO import StringIO
import sys, getopt
import re

usage = 'shift-gcode.py -x <X shift> -y <Y shift> -f <file>'

inFile = ''
xShift = 0
yShift = 0

try:
	opts, args = getopt.getopt(sys.argv[1:],"hx:y:f:")
except getopt.GetoptError:
	print usage
	sys.exit(2)
for opt, arg in opts:
	if opt == '-h':
		print usage
		sys.exit(2)
	elif opt in ("-x"):
		xShift = int(arg)
	elif opt in ("-y"):
		yShift = int(arg)
	elif opt in ("-f"):
		inFile = arg

fptr = open(inFile, 'r') 

lines = fptr.readlines()

for line in lines:
	if(re.match("G1", line)):
		xmatch = re.search(r"X(-?\d+\.?\d*)", line)
		if(xmatch):
			x = float(xmatch.group(1))
			line = line.replace(xmatch.group(0), "X" + str(x + xShift))
		ymatch = re.search(r"Y(-?\d+\.?\d*)", line)
		if(ymatch):
			y = float(ymatch.group(1))
			line = line.replace(ymatch.group(0), "Y" + str(y + yShift))
	
	sys.stdout.write(line)
	

#	words = line.split()
#	for word in words:
#		if re.match("X"):
			


