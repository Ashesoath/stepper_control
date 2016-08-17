#!/usr/bin/env python
#****************************************************************************
#  Copyright, 2016, SYSTEC electronic GmbH, D-08468 Heinsdorfergrund, Am Windrad 2
#  www.systec-electronic.com
#
#  Project: How to manage a stepper motor via ROS?
#
#  Description:  The Subscriber Node for system with 12V Stepper motor. Receive the data. Starting the motor in the specified direction.
#
#  -------------------------------------------------------------------------
#
#  Author: 		Vladyslav Tkachenko
#  Revision: 	1.0  
#  Date: 		2016/08/17 14:59:10
#
#  -------------------------------------------------------------------------
#
#  Revision History:
#
#  2016/08/01 -rs:   Initial Implementation
#
#  -------------------------------------------------------------------------
#  
#  All rights reserved.
#
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
#  POSSIBILITY OF SUCH DAMAGE.*/
#****************************************************************************/

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
import os

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
# Set up the pins what will need for us
GPIO.setup(16, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
p = GPIO.PWM(16,500)

# The function is the "callback" that handles the messages as they come in: 
def callback(msg):
	print msg.data
	if(msg.data == "clockwise"):
		# Send to the direction pin the True value
		GPIO.output(18, True)
		steps=0
		# 1000 is the 2.5 rotation for this stepper(12 V)
                while steps<1000:
                        p.start(1)
                        steps+=1
                        time.sleep(0.01)
                p.stop()

	elif(msg.data == "counterclockwise"):
		# Send to the direction pin the False value
		GPIO.output(18, False)
		steps=0
                while steps<1000:
                	p.start(1)
			steps+=1
                	time.sleep(0.01)
	        p.stop()

rospy.init_node('Stepper_Subscriber')
sub = rospy.Subscriber('rotation', String, callback)

rospy.spin()
