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

FIRMWARE = /opt/Digi/XCTU-NG/radio_firmwares/XB3-24A/XB3-24A_200D-th.xml
PROFILE_XBP24 = xbp24_10ef_cst.xpro
PROFILE_XB3 = xb3j_200D_cst.xml

BAUD = 9600
NEWBAUD = 115200
PORT = /dev/ttyUSB1

help:
	@echo "make profile ..... upload profile"
	@echo "make verify ...... verify profile"

firmware:
	/opt/Digi/XCTU-NG/XCTUcmd update_firmware -f $(FIRMWARE) -p $(PORT) -b $(BAUD) -v

profileXB3: $(PROFILE)
	/opt/Digi/XCTU-NG/XCTUcmd load_profile -f $(PROFILE_XB3) -p $(PORT) -b $(BAUD) -v

profileXB3_115k: $(PROFILE)
	/opt/Digi/XCTU-NG/XCTUcmd load_profile -f $(PROFILE_XB3) -p $(PORT) -b $(NEWBAUD) -v

profilePro: $(PROFILE)
	/opt/Digi/XCTU-NG/XCTUcmd load_profile -f $(PROFILE_XBP24) -p $(PORT) -b $(BAUD) -F -v

verify: 
	/usr/bin/python verify.py $(PORT) $(NEWBAUD)
