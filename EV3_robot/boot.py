# Write your code here :-)
# SPDX-FileCopyrightText: 2017 Limor Fried for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""CircuitPython Essentials Storage logging boot.py file"""
import board
import digitalio
import storage
import time
import os
import sys



storage.remount("/", disable_concurrent_write_protection = True)
f = open('temp.py', "r")

content = f.read()
f.close()
fin = open('main1.py', 'w')
fin.write(content)
fin.flush()
time.sleep(2)
fin.close()

