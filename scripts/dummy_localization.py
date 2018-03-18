#!/usr/bin/env python

import roslib
import rospy
import math
import tf
import tf2_ros
from geometry_msgs.msg import Pose, Quaternion, Point, TransformStamped
from nav_msgs.msg import Odometry
class DummyLocalization:

  def handleBPGT(self,bpgt):
    t=TransformStamped()
    t.header.stamp=rospy.Time.now()
    t.header.frame_id="map"
    t.child_frame_id="odom"
    t.transform.translation.x=bpgt.pose.pose.position.x+self.sx
    t.transform.translation.y=bpgt.pose.pose.position.y+self.sy
    t.transform.translation.z=0.0
    q=bpgt.pose.pose.orientation
    qp=[q.x,q.y,q.z,q.w]
    yaw=tf.transformations.euler_from_quaternion(qp)[2]+self.st
    q=tf.transformations.quaternion_from_euler(0,0,yaw)
    t.transform.rotation.x=q[0]
    t.transform.rotation.y=q[1]
    t.transform.rotation.z=q[2]
    t.transform.rotation.w=q[3]
    self.tf_broadcaster.sendTransform(t)


  def __init__(self):
    rospy.init_node('dummy_localization')

    startPos=rospy.get_param("robot_start")
    print(startPos)
    self.sx=float(startPos[0])
    self.sy=float(startPos[1])
    self.st=float(startPos[2])

    self.tf_broadcaster=tf2_ros.TransformBroadcaster()

    rospy.Subscriber("/odom",Odometry,self.handleBPGT)

if __name__=='__main__':
  d=DummyLocalization()
  rospy.spin()
