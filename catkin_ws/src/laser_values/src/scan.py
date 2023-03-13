#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    # values at 0 degree
    print msg.ranges[0]
    # values at 90 degree
    print msg.ranges[360]
    # values at 180 degree 
    print msg.ranges[719]
    print('--------------')
    print len(msg.ranges)
    print('--------------')

rospy.init_node('scan_values')
sub = rospy.Subscriber("/hsrb/base_scan", LaserScan, callback)
rospy.spin()
