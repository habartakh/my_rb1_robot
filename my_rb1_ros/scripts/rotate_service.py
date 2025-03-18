#! /usr/bin/env python

import rospy
import math
from my_rb1_ros.srv import Rotate, RotateResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class RotateService :
    def __init__(self) :
        self.k_p = 0.5 
        self.rotate_cmd = Twist()
        self.current_yaw = 0.0 
        self.rate = rospy.Rate(20)
        self.ctrl_c = False
        self.rotate_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odometry_callback )
        self.rotate_service = rospy.Service('/rotate_robot', Rotate , self.service_callback)
        self.response = RotateResponse()
        self.response.result = "Rotation not successful"
        rospy.loginfo("Service Ready")

    def odometry_callback(self, msg):
        # get the current yaw of robot
        # computation of the yaw from: 
        # https://www.theconstruct.ai/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/ 
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _,_,self.current_yaw = euler_from_quaternion (orientation_list)
        # print ("Curent yaw is : ", self.current_yaw)


    def service_callback(self, request): 
        rospy.loginfo("Service requested")
        # rotate the robot
        self.rotate_rb1(request.degrees)
        rospy.loginfo("Service completed")

        #stop the robot after rotation
        self.stop_rb1()

        #test if the rotation was successful or not 
        self.response.result = "Rotation successful"
        return self.response

    # Compute the duration necessary for the robot to reach desired angle
    # then rotate the robot around z-axis for that duration
    def rotate_rb1(self, target_degrees):
        rospy.loginfo ("Rotation function called")
        target_rad = self.current_yaw + (target_degrees * math.pi / 180) 

        angle_diff = target_rad - self.current_yaw
        
        # Rotate robot using cmd_vel messages
        self.rotate_cmd.linear.x = 0  # ensure there are not drifting velocities
        self.rotate_cmd.linear.y = 0 
        self.rotate_cmd.angular.z = 0.35 if angle_diff > 0 else -0.35

        # Duration to rotate (time = angle / angular speed)
        duration = abs(angle_diff) / abs (self.rotate_cmd.angular.z)


        # Rotate until the target angle is reached
        start_time = rospy.get_time()

        while rospy.get_time() - start_time < duration:
            self.publish_once_in_cmd_vel(self.rotate_cmd)
            self.rate.sleep()      


    def stop_rb1(self):
        # stop the rotation of the robot
        self.rotate_cmd.linear.x = 0 
        self.rotate_cmd.linear.y = 0 
        self.rotate_cmd.angular.z = 0
        self.publish_once_in_cmd_vel(self.rotate_cmd)
        rospy.loginfo("Robot stopped")

    def publish_once_in_cmd_vel(self, cmd):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.rotate_publisher.get_num_connections()
            # print("connections : ", connections)
            if connections > 0:
                self.rotate_publisher.publish(cmd)
                # rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()


if __name__ == '__main__' : 
    rospy.init_node('rotate_service')
    rotate_service = RotateService()
    rospy.spin()