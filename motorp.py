#!/usr/bin/env python
#****************************************************************************
#  Copyright, 2016, SYSTEC electronic GmbH, D-08468 Heinsdorfergrund, Am Windrad 2
#  www.systec-electronic.com
#
#  Project: How to manage a stepper motor via ROS?
#  Description:  The Publisher Node. Sending direction data for rotation. The user enters the data via a query  
#  -------------------------------------------------------------------------
#  Author: 		Vladyslav Tkachenko
#  Revision: 	1.0  
#  Date: 		2016/08/17 14:56:09
#  -------------------------------------------------------------------------
#  Revision History:
#  2016/08/01 -rs:   Initial Implementation
#  -------------------------------------------------------------------------
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
# In this case, we are going to use a String, 
# defined in the ROS standart message pakage.
# Sinse we are using a message from another package, 
# we have to tell the ROS build system about this by adding
# a dependency to our package.xml
from std_msgs.msg import String 

rospy.init_node('Stepper_Publisher')	# Initializing the node
pub = rospy.Publisher('rotation', String, queue_size=10)

while not rospy.is_shutdown():		# The function will return True if the node is ready to be shut down and False otherwise
	# This node publishes the input data on the topic "rotation"
	rate = rospy.Rate(1)
	data = raw_input('Enter clockwise or counterclockwise: ')
	if data == "clockwise" || "counterclockwise":
		pub.publish(data)
		rate.sleep()
