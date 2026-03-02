#!/usr/bin/env python
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
import threading

def publish_odom(robot_id, tf_buffer):
    pub = rospy.Publisher(f"/{robot_id}/odom", Odometry, queue_size=10)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform("map", f"{robot_id}/base_link", rospy.Time(0), rospy.Duration(1.0))
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.child_frame_id = f"{robot_id}/base_link"
            odom.pose.pose.position = trans.transform.translation
            odom.pose.pose.orientation = trans.transform.rotation
            odom.pose.covariance = [0] * 36
            odom.twist.covariance = [0] * 36
            pub.publish(odom)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("tf_to_odom_node") 

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    robot_ids = ['p1', 'p2', 'p3', 'p4']

    for rid in robot_ids:
        threading.Thread(target=publish_odom, args=(rid, tf_buffer)).start()

    rospy.spin()

