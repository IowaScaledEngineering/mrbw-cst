#!/usr/bin/python

from cStringIO import StringIO
import sys, getopt
import re

usage = 'override-fan.py -a <start z> -b <end z> -s <fanSpeed> -f <file>'

inFile = ''
zStart = 0
zEnd = 0

try:
	opts, args = getopt.getopt(sys.argv[1:],"ha:b:s:f:")
except getopt.GetoptError:
	print usage
	sys.exit(2)
for opt, arg in opts:
	if opt == '-h':
		print usage
		sys.exit(2)
	elif opt in ("-a"):
		zStart = float(arg)
	elif opt in ("-b"):
		zEnd = float(arg)
	elif opt in ("-s"):
		fanSpeed = int(arg)
	elif opt in ("-f"):
		inFile = arg

fptr = open(inFile, 'r') 

lines = fptr.readlines()

inRange = False
inLayerChange = False
skipLine = False
lastFan = "; No previous fan speed!!\n"

for line in lines:
	if(re.search("BEFORE_LAYER_CHANGE", line)):
		inLayerChange = True
	if(re.search("AFTER_LAYER_CHANGE", line)):
		inLayerChange = False
	
	if(inLayerChange):
		if(re.match(";[0-9]", line)):
			zHeight = float(line[1:])
#			print str(zStart) + " : " + str(zHeight) + " : " + str(zEnd) + " : " + str(inRange)
			if((zStart <= zHeight) and (zHeight <= zEnd)):
				inRange = True
				sys.stdout.write("M106 S" + str(fanSpeed) + "\n")
			else:
				if(inRange):
					sys.stdout.write(lastFan)
				inRange = False
	elif(re.match("^M106", line)):
		if(inRange):
			sys.stdout.write("M106 S" + str(fanSpeed) + "\n")
			skipLine = True
		lastFan = line

	if(not skipLine):
		sys.stdout.write(line)
	skipLine = False

