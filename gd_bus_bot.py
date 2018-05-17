#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
import roslib
from kobuki_msgs.msg import ButtonEvent
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import time

have_packages_on_bot = False #Are there packages on the bot?
waiting_for_input = True #waiting the customer
back_to_origin = True
cannot_move_until_b0_is_pressed = False #TurtleBot should stay still until B0 is pressed
class GoToPose():


    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))
	#monitor kobuki's button events
	rospy.Subscriber("/mobile_base/events/button",ButtonEvent,self.ButtonEventCallback)

    def goto(self, pos, quat):
	#if someone is currently putting or taking away packages, don't move!
	if(cannot_move_until_b0_is_pressed):
		rospy.loginfo("Waiting for button B0 to be pressed.")
		time.sleep(2)
		return True
		
        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result
        
	
    def ButtonEventCallback(self,data):
	global waiting_for_input
	global cannot_move_until_b0_is_pressed
	if ( data.button == ButtonEvent.Button0 ) :
	    rospy.loginfo("B0 has been pressed.")
	    cannot_move_until_b0_is_pressed = False
	    waiting_for_input = True
			
			
			
    def shutdown(self):
        #if self.goal_sent:
        #self.move_base.cancel_goal()
	rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        position_a1 = {'x': 6.13, 'y' : 1.33}
        quaternion_a1 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_a2 = {'x': 6.05, 'y' : -1.47}
        quaternion_a2 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_b1 = {'x': -0.0338, 'y' : 2.82}
        quaternion_b1 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_b2 = {'x': 2.95, 'y' : 1.75}
        quaternion_b2 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}    
        position_o = {'x': 0.071, 'y' : 0.039}
        quaternion_o = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}    
	while not rospy.is_shutdown():		
		if waiting_for_input:
			if not have_packages_on_bot:
				order = raw_input('*Call for a GdBot, please choose the place A1 or A2; Send the GdBot back, please enter back:')
				rospy.loginfo("Go to %s", order)
				if order == 'A1':
					waiting_for_input = False
					success = navigator.goto(position_a1, quaternion_a1)
					if success:
						print("*Yea, reached A1! If the packages are ready, press B0 to allow GdBot to continue.")
						have_packages_on_bot = True
						back_to_origin = False
						#tell TurtleBot not to move until the customer presses B0
						cannot_move_until_b0_is_pressed = True

					else:
						rospy.loginfo("The base failed to reach the desired pose, send the bot back.")
						back_to_origin = navigator.goto(position_o, quaternion_o)
						if back_to_origin:
							print('*The GdBot is at origin place.')
					# Sleep to give the last log messages time to be sent
					rospy.sleep(1)

				elif order == 'A2':
					waiting_for_input = False
					success = navigator.goto(position_a2, quaternion_a2)
					if success:
						print('*Yea, reached A2! If the packages are ready, press B0 to allow GdBot to continue.')
						have_packages_on_bot = True
						back_to_origin = False
						#tell TurtleBot not to move until the customer presses B0
						cannot_move_until_b0_is_pressed = True

					else:
						rospy.loginfo("The base failed to reach the desired pose, send the bot back.")
						back_to_origin = navigator.goto(position_o, quaternion_o)
						if back_to_origin:
							print('*The GdBot is at origin place.')
					rospy.sleep(1)
					
							
				elif order == 'back':
					if back_to_origin:
						print('*The GdBot has been at the origin place.')
					else:
						rospy.loginfo("Begin sending the GdBot back.")
						back_to_origin = navigator.goto(position_o, quaternion_o)
						if back_to_origin:
							print('*The GdBot is at origin place.')
					rospy.sleep(1)
					
				else:
					print('*Please choose from A1 and A2 or send the GdBot back(enter back):')
				continue
			
			if have_packages_on_bot:
				order = raw_input('*Where do you want to put your packages? Please input B1 or B2:')
				rospy.loginfo("Go to %s", order)
				if order == 'B1':
					waiting_for_input = False
					success = navigator.goto(position_b1, quaternion_b1)
					if success:
						print("*Yea, reached B1! If the packages have been taken away, press B0 to allow GdBot to continue.")
						have_packages_on_bot = False
						back_to_origin = False
						#tell TurtleBot not to move until the customer presses B0
						cannot_move_until_b0_is_pressed = True

					else:
						rospy.loginfo("The base failed to reach the desired pose")
						back_to_origin = navigator.goto(position_o, quaternion_o)
						if back_to_origin:
							print('*The GdBot is at origin place.')

					rospy.sleep(1)
				elif order == 'B2':
					waiting_for_input = False
					success = navigator.goto(position_b2, quaternion_b2)
					if success:
						print("*Yea, reached B2! If the packages have been taken away, press B0 to allow GdBot to continue.")
						have_packages_on_bot = False
						back_to_origin = False
						#tell TurtleBot not to move until the customer presses B0
						cannot_move_until_b0_is_pressed = True

					else:
						rospy.loginfo("The base failed to reach the desired pose")
						back_to_origin = navigator.goto(position_o, quaternion_o)
						if back_to_origin:
							print('*The GdBot is at origin place.')
					rospy.sleep(1)
				else:
					print('*Please choose from B1 and B2:')
				continue
		rospy.sleep(1)			

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

