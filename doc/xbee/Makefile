#*************************************************************************
#Title:    XBee Makefile
#Authors:  Michael Petersen <railfan@drgw.net>
#          Nathan Holmes <maverick@drgw.net>
#File:     XBee Makefile
#License:  GNU General Public License v3
#
#LICENSE:
#    Copyright (C) 2018 Nathan Holmes and Michael Petersen
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 3 of the License, or
#    any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#    
#    You should have received a copy of the GNU General Public License along 
#    with this program. If not, see http://www.gnu.org/licenses/
#    
#*************************************************************************

PROFILE = xbp24_10ef_cst

BAUD = 9600
NEWBAUD = 115200
PORT = /dev/ttyUSB0

help:
	@echo "make profile ..... upload profile"
	@echo "make verify ...... verify profile"

profile: $(PROFILE).xml
	/opt/Digi/XCTU-NG/XCTUcmd load_profile -f $(PROFILE).xml -p $(PORT) -b $(BAUD) -F -v

verify: 
	/usr/bin/python verify.py $(PORT) $(NEWBAUD)
