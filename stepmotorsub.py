#!/usr/bin/env python
# SYS TEC electronic
# Tkachenko Vladyslav

# Import libraries
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
import os
import sys

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
steppins = [12,16,18,22]	# Define GPIO signals
# Set all pins as output
for pin in steppins:
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, False)

# Define advanced sequence
Seq=[[1,0,0,1],[1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],[0,0,1,0],[0,0,1,1],[0,0,0,1]]
StepCount = len(Seq)


def callback(msg):
	print msg.data
	if(msg.data == "clockwise"):
		Direction = 1
		StepCounter = 0
		Step = 0
		while Step < 4096:
			for pin in range(0,4):
				# Get the GPIO
				xpin=steppins[pin]
				if(Seq[StepCounter][pin] != 0):
					GPIO.output(xpin,True)
				else:
					GPIO.output(xpin,False)

			StepCounter += Direction

			if(StepCounter >= StepCount):
				StepCounter = 0
			if (StepCounter < 0):
				StepCounter = StepCount + Direction
			time.sleep(0.001)
			Step += 1

	elif(msg.data == "counterclockwise"):
                Direction = -1
                StepCounter = 0
                Step = 0
                while Step < 4096:
                        for pin in range(0,4):
                                xpin=steppins[pin]
                                if(Seq[StepCounter][pin] != 0):
                                        GPIO.output(xpin,True)
                                else:
                                        GPIO.output(xpin,False)

                        StepCounter += Direction

                        if(StepCounter >= StepCount):
                                StepCounter = 0
                        if (StepCounter < 0):
                                StepCounter = StepCount + Direction
                        time.sleep(0.001)
                        Step += 1

rospy.init_node('Stepper_28BYJ_Sub')
sub = rospy.Subscriber('rotation', String, callback)

rospy.spin()
