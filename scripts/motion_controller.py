#!/usr/bin/env python

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion, Twist, PointStamped
from std_msgs.msg import ColorRGBA, String, Header


class MotionController:
    """
    This is the Node responsible for capturing Points published
    by the PathPlanner and getting the robot to those points
    """
    def __init__(self):
        rospy.init_node('motion_controller')
        check_rate = rospy.Rate(1)
        while not rospy.has_param('path_data'):
            check_rate.sleep()
        self.path_points = rospy.get_param('path_data')[1:]  # don't need the origin
        self.counter = 0
        self.next_point = Point(*self.path_points[self.counter])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.listener = tf.TransformListener()
        drive_rate = rospy.Rate(60)
        while self.path_points:
            self.drive_along_path()
            drive_rate.sleep()
        rospy.spin()

    def debug(self):
        try:
            sig =  rospy.Time.now()
            self.listener.waitForTransform('/real_robot_pose', '/map',sig, rospy.Duration(5.0))
            print self.listener.lookupTransform('/real_robot_pose', '/map', sig)
        except Exception as e:
            print e

    def drive_along_path(self):
        #transform next_point to real_robot_pose
        while self.counter <= len(self.path_points):
            try:
                sig =  rospy.Time.now()
                ps = PointStamped(header=Header(stamp=sig, frame_id='map'), point=self.next_point)
                self.listener.waitForTransform('/real_robot_pose', '/map',sig, rospy.Duration(1.0))
                tf_point = self.listener.transformPoint('/real_robot_pose', ps)  # this is what's actually used to move
                theta = math.atan2(tf_point.point.y, tf_point.point.x)
                if abs(math.hypot(tf_point.point.x, tf_point.point.y)) <0.05:  #  if robot is on point, update next
                    self.publish_twist_message(0,0)
                    self.next_point = Point(*self.path_points[self.counter])
                    self.counter += 1
                if abs(theta)>0.01:  # this is the margin for error I'm allowing.
                    self.publish_twist_message(0,theta)
                else:
                    self.publish_twist_message(math.hypot(tf_point.point.x, tf_point.point.y), 0)
            except Exception as e:
                return

    def drive_with_amcl(self):
        pass

    def debug_mover(self):
        redundancy = 3  # ROS doesn't like when you publish one message
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.1
        for i in range(redundancy):
            self.pub.publish(msg)

    def publish_twist_message(self, linear, angular):
        redundancy = 1  # ROS doesn't like when you publish one message
        msg = Twist()
        msg.linear.x = linear/redundancy
        msg.angular.z = angular/redundancy
        for i in range(redundancy):
            self.pub.publish(msg)

    def update_next_point(self):
        pass

if __name__ == '__main__':
    MotionController()
