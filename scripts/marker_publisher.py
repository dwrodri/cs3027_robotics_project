#!/usr/bin/env python

import rospy
import roslib
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from std_msgs.msg import ColorRGBA

class RobotAndGoalMarkerGenerator:
    def __init__(self):
        rospy.init_node('robot_marker_publisher')
        rate = rospy.Rate(2)
        self.pub = rospy.Publisher('/marker_channel', Marker, queue_size=200)
        goal_dict = {key: value for key, value in rospy.get_param('/').iteritems() if 'goal' in key}  # get goals from param server
        self.goal_list = goal_dict.values()
        map(lambda x: x.append(0.1/2), self.goal_list)  # add z value
        bpgt_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, self.publish_true_robot_marker)
        odom_sub = rospy.Subscriber('/odom', Odometry, self.publish_uncertain_robot_marker)
        rospy.spin()

    def publish_true_robot_marker(self, odom_msg):
        """
        gets robot pose from base_pose_ground_truth and publishes a blue CUBE marker to Pub
        :param odom_message: Odometry message from base_pose_ground_truth
        """
        marker = Marker()
        marker.header.frame_id = 'real_robot_pose' #look, I'm totally using that transform from step 1
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'assessment'
        marker.type = Marker.CUBE
        marker.id = 0
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.pose = Pose(Point(z=0.05), odom_msg.pose.pose.orientation)  #put it at the origin of real_robot_pose
        marker.color = ColorRGBA(b=1, a=1)
        self.pub.publish(marker)
        self.publish_goal_markers(self.goal_list)  # do this here because rviz sucks

    def publish_uncertain_robot_marker(self, odom_msg):
        """
        generate a pose using robot_pose parameter ,and transform it using /odom
        :param odom_msg: Odometry message coming from /odom
        """

        point = Point(*rospy.get_param('robot_start'))  # get x, y, z of robot_start
        point.x += odom_msg.pose.pose.position.x  # update position based on /odom data
        point.y += odom_msg.pose.pose.position.y
        point.z += odom_msg.pose.pose.position.z + (0.1/2)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'assessment'
        marker.type = Marker.CUBE
        marker.id = 1
        marker.scale = Vector3(0.15, 0.15, 0.15)
        marker.pose = Pose(point, odom_msg.pose.pose.orientation)  # I know getting my quaternion this way is kinda cheating, but whatever
        marker.color = ColorRGBA(r=0, b=1, a=0.7)
        self.pub.publish(marker)


    def publish_goal_markers(self, goals):
        goals = [Point(*x) for x in goals]
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'assessment'
        marker.type = Marker.SPHERE_LIST
        marker.id = 2
        marker.scale = Vector3(0.1, 0.1, 0.1)  # just a little bit smaller than the robot, so it can drive over them
        marker.points = goals
        marker.color = ColorRGBA(1.0, 0.5, 0, 255)
        self.pub.publish(marker)

if __name__ == '__main__':
    RobotAndGoalMarkerGenerator()
