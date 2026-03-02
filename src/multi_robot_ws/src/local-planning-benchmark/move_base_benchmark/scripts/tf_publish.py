#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def p1_odom_callback(msg):
    odom_msg = Odometry()
    odom_msg.header = msg.header
    odom_msg.pose = msg.pose

    # 创建tf广播器
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     "p1/base_link",
                     "map")
    
def p2_odom_callback(msg):
    odom_msg = Odometry()
    odom_msg.header = msg.header
    odom_msg.pose = msg.pose

    # 创建tf广播器
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,                     
                     "p2/base_link",
                     "map")

def p3_odom_callback(msg):
    odom_msg = Odometry()
    odom_msg.header = msg.header
    odom_msg.pose = msg.pose

    # 创建tf广播器
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     "p3/base_link",
                     "map")
                    
def p4_odom_callback(msg):
    odom_msg = Odometry()
    odom_msg.header = msg.header
    odom_msg.pose = msg.pose

    # 创建tf广播器
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     "p4/base_link",
                     "map")
    
def p5_odom_callback(msg):
    odom_msg = Odometry()
    odom_msg.header = msg.header
    odom_msg.pose = msg.pose

    # 创建tf广播器
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     "p5/base_link",
                     "map")
    
def p6_odom_callback(msg):
    odom_msg = Odometry()
    odom_msg.header = msg.header
    odom_msg.pose = msg.pose

    # 创建tf广播器
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     "p6/base_link",
                     "map")

if __name__ == '__main__':
    rospy.init_node('tf_publisher')
    rospy.Subscriber("/p1/base_pose_ground_truth", Odometry, p1_odom_callback)
    rospy.Subscriber("/p2/base_pose_ground_truth", Odometry, p2_odom_callback)
    rospy.Subscriber("/p3/base_pose_ground_truth", Odometry, p3_odom_callback)
    rospy.Subscriber("/p4/base_pose_ground_truth", Odometry, p4_odom_callback)
    rospy.Subscriber("/p5/base_pose_ground_truth", Odometry, p5_odom_callback)
    rospy.Subscriber("/p6/base_pose_ground_truth", Odometry, p6_odom_callback)
    #rate set to 10
    rate = rospy.Rate(30)
    rospy.spin()
