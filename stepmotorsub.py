#!/usr/bin/env python
#****************************************************************************
#  Copyright, 2016, SYSTEC electronic GmbH, D-08468 Heinsdorfergrund, Am Windrad 2
#  www.systec-electronic.com
#
#  Project: How to manage a stepper motor via ROS?
#  Description:  The Subscriber Node for system with Step motor 28BYJ-48. Receive the data. Starting the motor in the specified direction.
#  -------------------------------------------------------------------------
#  All rights reserved.
#  Redistribution and use in source and binary forms, with or without 
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, 
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice, 
#     this list of conditions and the following disclaimer in the documentation 
#	 and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its contributors 
#     may be used to endorse or promote products derived from this software 
#	 without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
#  POSSIBILITY OF SUCH DAMAGE.
# ****************************************************************************
# I want to express my gratitude to author Matt from web-site http://www.raspberrypi-spy.co.uk/author/matt/.
# Lines of code have been taken from this site, which helped in the implementation of the system.
# Thank you for a wonderful code, I recommend to visit his page on the web-site http://www.raspberrypi-spy.co.uk/.
# ****************************************************************************
import time
import os
import sys
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
steppins = [12,16,18,22]	# Define GPIO signals
# Set all pins as output
for pin in steppins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)
# Define advanced sequence
seq = [[1,0,0,1],[1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],[0,0,1,0],[0,0,1,1],[0,0,0,1]]
step_count = len(seq)

def sequence(step_counter):
    for pin in range(0,4):
        # Get the GPIO
        xpin=steppins[pin]
        if seq[step_counter][pin] != 0:
            GPIO.output(xpin,True)
        else:
            GPIO.output(xpin,False)

def steps(direction):
    step = 0
    step_counter = 0
    while step < 4096:
        sequence(step_counter)
        step_counter += direction
        if step_counter >= step_count:
            step_counter = 0
        if step_counter < 0:
            step_counter = step_count + direction
        time.sleep(0.001)
        step += 1

def callback(msg):
    print msg.data
    if msg.data == "clockwise":
        direction = 1
    else:
        direction = -1
    steps(direction)

rospy.init_node('Stepper_28BYJ_Sub')
sub = rospy.Subscriber('rotation', String, callback)
rospy.spin()
