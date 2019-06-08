#!/usr/bin/python

from cStringIO import StringIO
import sys, getopt
import re

usage = 'adjust-time.py -f <file>'

inFile = ''
instances = 0

try:
	opts, args = getopt.getopt(sys.argv[1:],"hf:")
except getopt.GetoptError:
	print usage
	sys.exit(2)
for opt, arg in opts:
	if opt == '-h':
		print usage
		sys.exit(2)
	elif opt in ("-f"):
		inFile = arg

fptr = open(inFile, 'r') 

lines = fptr.readlines()

instances = 1
instance = 0
p_old = 0
r_max = 0

# Get stats
for line in lines:
	match = re.search(r"M73 P(\d+) R(\d+)", line)
	if(match):
		p = int(match.group(1))
		r = int(match.group(2))
		r_max = max(r, r_max)

		# Detect next instance when percentage resets to 0
		if (p < p_old):
			instances = instances + 1
		p_old = p

p_old = 0

for line in lines:
	match = re.search(r"M73 P(\d+) R(\d+)", line)
	if(match):
		p = int(match.group(1))
		r = int(match.group(2))
		r_max = max(r, r_max)

		# Detect next instance when percentage resets to 0
		if (p < p_old):
			instance = instance + 1
		p_old = p

		# Percentage as per instance, time as total time
		line = "M73 P" + str(p) + " R" + str((r_max * instances) - r_max*(instance) - (r_max - r)) + "\n"
#		line = "M73 P" + str((p + 100*instance)/instances) + " R" + str((r_max * instances) - r_max*(instance) - (r_max - r)) + "\n"

	sys.stdout.write(line)


