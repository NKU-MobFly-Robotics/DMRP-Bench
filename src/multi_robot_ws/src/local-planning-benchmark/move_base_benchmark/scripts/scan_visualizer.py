#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

def scan_callback(scan_msg):
    marker = Marker()
    marker.header = scan_msg.header
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    
    angle = scan_msg.angle_min
    for r in scan_msg.ranges:
        if r < scan_msg.range_max and r > scan_msg.range_min:
            p = Point()
            p.x = r * math.cos(angle)
            p.y = r * math.sin(angle)
            marker.points.append(p)
        angle += scan_msg.angle_increment
    
    pub.publish(marker)
    
    # 打印扫描信息，帮助调试
    rospy.loginfo("Scan info - angle_min: %f, angle_max: %f, points: %d", 
                 scan_msg.angle_min, scan_msg.angle_max, len(marker.points))

if __name__ == '__main__':
    rospy.init_node('scan_visualizer')
    pub = rospy.Publisher('/p1/scan_markers', Marker, queue_size=1)
    sub = rospy.Subscriber('/p1/RosAria/scan', LaserScan, scan_callback)
    rospy.loginfo("Scan visualizer node started")
    rospy.spin()
