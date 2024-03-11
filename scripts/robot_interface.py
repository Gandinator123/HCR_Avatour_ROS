"""
Personal Robotics Laboratory - Imperial College London, 2021
Authors:
    - Rodrigo Chacon Quesada (rac17@ic.ac.uk)
    - Haining Luo (haining.luo18@imperial.ac.uk)
    - Cedric Goubard (c.goubard21@imperial.ac.uk)


Inspired from https://github.com/redragonx/can2RNET
Original code was modified to:
    - make it work with ROS Noetic
    - adapt to the specificities of our joysticks
    - work in our dockerised setup

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
from ctypes import ArgumentError
from time import sleep

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from typing import Tuple
import threading
from typing import List 
import time 


class JoyStick:
    def __init__(self):
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.modes = ["standing", "walking"]
        self.mode = self.modes[0] ## init the enum
        self.start()
                
    def close(self):
        self.stop = True
        rospy.loginfo("Joystick closed")
        
    def start(self):
        self.stop = False
        rospy.loginfo("Joystick started")


class robot_interface:

    def __init__(self,):
        
        ######################
        ##### Substribers #####
        ######################
        self.sub_VR_joy_left = rospy.Subscriber("/VR/joy_left", Joy, self.call_back_VR_joy_left, queue_size=1)
        self.sub_VR_joy_right = rospy.Subscriber("/VR/joy_right", Joy, self.call_back_VR_joy_right, queue_size=1)
        self.sub_VR_joy_mode = rospy.Subscriber("/VR/joy_mode", String, self.call_back_VR_mode, queue_size=1)
        rospy.loginfo("VR joystick subscriber created")        
        
        self.joystick = JoyStick()
        
        
        ### publish back to the wheelchair
        ## ROS publisher to send joystick data
        self.robot_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
        rospy.loginfo('Final velocity command publisher created')
        
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = 0
        
        self.move = False
        

       
    
    #########################
    ##### Methods #######
    ######################### 

    def run(self):
        self.timer_left_joy = threading.Timer(0.5, self.timeout_left_joy)
        self.timer_left_joy.start()
        self.timer_right_joy = threading.Timer(0.5, self.timeout_right_joy)
        self.timer_right_joy.start()
        while not rospy.is_shutdown():
            try:
                '''
                DO STUFF HERE to process
                '''
                self.move = self.joystick.mode == "standing"
                rospy.loginfo("Mode: " + str(self.joystick.mode))
                rospy.loginfo("Moving: " + str(self.joystick.left_x) + " " + str(self.joystick.left_y) + " " + str(self.joystick.right_x) + " " + str(self.joystick.right_y))
                rospy.loginfo("Time: " + str(time.ctime(time.time())))
                ## P3AT robot code
                self.cmd_vel.linear.x = self.joystick.left_y
                self.cmd_vel.angular.z = self.joystick.right_x
                # rospy.loginfo("Sending to p3AT")
                self.robot_pub.publish(self.cmd_vel)
                    
            except KeyboardInterrupt:
                self.shutdown()
                rospy.loginfo("Connection Off")
                rospy.loginfo("Socket down")
                break
            
    def shutdown(self):
        rospy.loginfo("Closing robot_interface node")
        rospy.signal_shutdown()
    

    def timeout_left_joy(self):
        self.joystick.left_x = 0
        self.joystick.left_y = 0
        rospy.loginfo("Timeout left joystick")
        
    def timeout_right_joy(self):
        self.joystick.right_x = 0
        self.joystick.right_y = 0
        rospy.loginfo("Timeout right joystick")

    ## NOTE: need to ensure linear.x and angular.z are set to 0 (from server function) when the joystick is not being used
    ## should be called when we receive a joystick message
    def call_back_VR_joy_left(self, joy_msg:  Joy):
        try:
            self.joystick.left_x = float(joy_msg.axes[0])
            self.joystick.left_y = float(joy_msg.axes[1])
            rospy.loginfo("Left Joystick X: " + str(self.joystick.left_x) + " Y: " + str(self.joystick.left_y))
            rospy.loginfo("Time robot interface: " + str(time.ctime(time.time())))
            self.timer_left_joy.cancel()
            self.timer_left_joy = threading.Timer(0.5, self.timeout_left_joy)
            self.timer_left_joy.start()
        except Exception as e:
            rospy.loginfo("Error in left joystick: " + str(e))
        
    def call_back_VR_joy_right(self, joy_msg:  Joy):
        try:
            self.joystick.right_x = float(joy_msg.axes[0])
            self.joystick.right_y = float(joy_msg.axes[1])
            rospy.loginfo("Right Joystick X: " + str(self.joystick.right_x) + " Y: " + str(self.joystick.right_y))
            rospy.loginfo("Time robot interface: " + str(time.ctime(time.time())))
            self.timer_right_joy.cancel()
            self.timer_right_joy = threading.Timer(0.5, self.timeout_right_joy)
            self.timer_right_joy.start()
        except Exception as e:
            rospy.loginfo("Error in right joystick: " + str(e))
        
    def call_back_VR_mode(self, mode: String):
        try:
            if mode.data in self.joystick.modes:
                self.joystick.mode = mode
                rospy.loginfo("Mode " + str(self.joystick.mode))
                rospy.loginfo("Time robot interface: " + str(time.ctime(time.time())))
            else: 
                rospy.loginfo("Invalid mode")
        except:
            rospy.loginfo("Error in mode: " + str(e))
                    
    def call_back_stop(self, stop: String):
        if stop.data == "stop":
            self.joystick.close()
            rospy.loginfo("Joystick closed")
        
        

if __name__ == "__main__":

    rospy.sleep(0.5)

    rospy.init_node('robot_interface')

    rmv = robot_interface()

    try:
        rmv.run()
    except KeyboardInterrupt:
        rmv.shutdown()
        rospy.loginfo("CAN disconnected")
        rospy.loginfo("Connection Off")
        rospy.loginfo("Socket down")

