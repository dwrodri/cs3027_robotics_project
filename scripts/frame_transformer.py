#!/usr/bin/env python
import rospy
import roslib
import tf
from geometry_msgs.msg import Pose, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
class FrameTransformer:
    def __init__(self):
        rospy.init_node('frame_transformer')
        self.broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber(
        '/base_pose_ground_truth',
        Odometry,
        self.handle_map
        )
        rospy.Subscriber(
        '/fake_localization',
        PoseWithCovarianceStamped,
        self.handle_localization
        )
        rospy.spin()

    def handle_map(self, odom_msg):
        self.broadcaster.sendTransform(
        (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0),
        [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w],
        rospy.Time.now(),
        "real_robot_pose",
        "map"
        )

    def handle_localization(self, pose_covar_msg):
        self.broadcaster.sendTransform(
        (pose_covar_msg.pose.pose.position.x, pose_covar_msg.pose.pose.position.y, 0),
        (pose_covar_msg.pose.pose.orientation.x, pose_covar_msg.pose.pose.orientation.y, pose_covar_msg.pose.pose.orientation.z, 1),
        pose_covar_msg.header.stamp,
        "statistical_robot_pose",
        "map"
        )


    def handle_laser_frame(self, laser_msg):
        pass

if __name__ == '__main__':
    FrameTransformer()
