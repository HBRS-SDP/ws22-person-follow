#!/usr/bin/python

import rospy

# Custom messages
from leg_tracker.msg import Person, PersonArray, Leg, LegArray 

# ROS messages
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# Standard python modules
import numpy as np
import random
import math
from scipy.optimize import linear_sum_assignment
import scipy.stats
import scipy.spatial
from geometry_msgs.msg import PointStamped, Point
from tmc_geometry_msgs.msg import Point2DStamped, Point2D
import tf
import copy
import timeit
import message_filters
import sys

# External modules
from pykalman import KalmanFilter # To install: http://pykalman.github.io/#installation

class KalmanMultiTracker:
    """
    Tracker for tracking all the people and objects
    """
    max_cost = 9999999

    def __init__(self):
        """
        Constructor
        """
        self.objects_tracked = []
        self.potential_leg_pairs = set()
        self.potential_leg_pair_initial_dist_travelled = {}
        self.people_tracked = []
        self.prev_track_marker_id = 0
        self.prev_person_marker_id = 0
        self.prev_time = None
        self.listener = tf.TransformListener()
        print('tf. TransformListener : ',tf.TransformListener())
        self.local_map = None
        self.new_local_map_received = True
        random.seed(1)

        # Get ROS params
        self.fixed_frame = rospy.get_param("fixed_frame", "map")
        self.max_leg_pairing_dist = rospy.get_param("max_leg_pairing_dist", 0.8)
        self.confidence_threshold_to_maintain_track = rospy.get_param("confidence_threshold_to_maintain_track", 0.1)
        self.publish_occluded = rospy.get_param("publish_occluded", True)
        self.publish_people_frame = rospy.get_param("publish_people_frame", self.fixed_frame)
        self.use_scan_header_stamp_for_tfs = rospy.get_param("use_scan_header_stamp_for_tfs", False)
        self.publish_detected_people = rospy.get_param("display_detected_people", False)
        self.dist_travelled_together_to_initiate_leg_pair = rospy.get_param("dist_travelled_together_to_initiate_leg_pair", 0.5)
        scan_topic = rospy.get_param("scan_topic", "base_scan");
        self.scan_frequency = rospy.get_param("scan_frequency", 7.5)
        self.in_free_space_threshold = rospy.get_param("in_free_space_threshold", 0.06)
        self.confidence_percentile = rospy.get_param("confidence_percentile", 0.90)
        self.max_std = rospy.get_param("max_std", 0.9)

        self.mahalanobis_dist_gate = scipy.stats.norm.ppf(1.0 - (1.0-self.confidence_percentile)/2., 0, 1.0)
        self.max_cov = self.max_std**2
        self.latest_scan_header_stamp_with_tf_available = rospy.get_rostime()
        print('ROS Parameters initialised :')
        print('M.D : ',self.mahalanobis_dist_gate)

        # ROS publishers
        print('ROS Pub begin..')
        print('PersonArray : ',PersonArray)
        print('Header: {}  and people : {}'.format(PersonArray.header,PersonArray.people))
        self.people_tracked_pub = rospy.Publisher('people_tracked', PersonArray, queue_size=300)
        print('People tracked_pub : ',self.people_tracked_pub)
        print('PersonArray before people detected')
        print('Header: {}  and people : {}'.format(PersonArray.header,PersonArray.people))
        self.people_detected_pub = rospy.Publisher('people_detected', PersonArray, queue_size=300)
        print('People detected_pub : ',self.people_detected_pub)
        print('Marker : ',Marker)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=300)
        print('marker_pub : ',self.marker_pub)
        print('LegArray  : ',LegArray) 
        self.non_leg_clusters_pub = rospy.Publisher('non_leg_clusters', LegArray, queue_size=300)
        print('non leg cluster_pub : ',self.non_leg_clusters_pub)

          # ROS subscribers
        print('ROS Subscriber begins ...')
        self.detected_clusters_sub = rospy.Subscriber('detected_leg_clusters', LegArray, self.detected_clusters_callback)
        print('detected cluster sub : ',self.detected_clusters_sub)
        print('Occupancy Grid : ',OccupancyGrid)
        self.local_map_sub = rospy.Subscriber('local_map', OccupancyGrid, self.local_map_callback)
        print('Local map sub : ',self.local_map_sub)





    def local_map_callback(self, map):
        """
        Local map callback to update our local map with a newly published one
        """
        self.local_map = map
        self.new_local_map_received = True
        print('local map : {} | new local map received : {}'.format(self.local_map,self.new_local_map_received))


    

    def detected_clusters_callback(self, detected_clusters_msg):    
        """
        Callback for every time detect_leg_clusters publishes new sets of detected clusters. 
        It will try to match the newly detected clusters with tracked clusters from previous frames.
        """
        # Waiting for the local map to be published before proceeding. This is ONLY needed so the benchmarks are consistent every iteration
        # Should be removed under regular operation
        if self.use_scan_header_stamp_for_tfs: # Assume <self.use_scan_header_stamp_for_tfs> means we're running the timing benchmark
            wait_iters = 0
            while self.new_local_map_received == False and wait_iters < 10:
                rospy.sleep(0.1)
                wait_iters += 1
            if wait_iters >= 10:
                rospy.loginfo("no new local_map received. Continuing anyways")
            else:
                self.new_local_map_received = False

        now = detected_clusters_msg.header.stamp
        print('detected cluster msg header (now var) : ',now)

        detected_clusters = []
        detected_clusters_set = set()
        for cluster in detected_clusters_msg.legs:
            new_detected_cluster = DetectedCluster(
                cluster.position.x, 
                cluster.position.y, 
                cluster.confidence, 
                in_free_space=self.how_much_in_free_space(cluster.position.x, cluster.position.y)
            )      
            if new_detected_cluster.in_free_space < self.in_free_space_threshold:
                new_detected_cluster.in_free_space_bool = True
            else:
                new_detected_cluster.in_free_space_bool = False
            detected_clusters.append(new_detected_cluster)
            detected_clusters_set.add(new_detected_cluster)  
        








if __name__ == '__main__':
    rospy.init_node('multi_person_tracker', anonymous=True)
    print('Rospy node initiated ')
    kmt = KalmanMultiTracker()
    print('KalmanMultiTracker done')
