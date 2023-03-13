# SDP
Software Development Project: Person Following.

This project is built off of: https://github.com/angusleigh/leg_tracker, which is in turn based on http://wiki.ros.org/leg_detector.

See the parent repositories for additional details on how the leg detection works and parameters that can be tuned.

# Prerequisites
Pykalman and Scipy are required to run this package.
- python3 -m pip install scipy
- python3 -m pip install pykalman

# Steps for use

1. Clone the git repo to your computer
2. Open a terminal window and navigation to where you cloned the repo
3. Navigate to the catkin workspace with: cd catkin_ws
4. Once inside the catkin workspace, clean it with: catkin clean
5. Catkin clean will prompt you with yN. You need to clean the entire workspace, including the devel and log folders, so type: y
6. Once the workspace is cleaned you need to build it. You can use: catkin build. If this fails to build, catkin clean the workspace, then try: catkin_make
7. After the workspace is built, you need to source the setup file with. You have to be within the catkin_ws folder for this to work. Type: source devel/setup.bash
8. To launch the leg tracker package, navigate to the demos folder with: cd src/leg_tracker/launch
9. Launch the joint_leg_tracker launch fil using $roslaunch joint_leg_tracker.launch

Using RVIZ
1. To manually configure RVIZ, first run a launch file and wait for RVIZ to open. If the LiDAR points and visualization markers are not appearing, but the laser scan and markers appear on the left-side pane in RVIZ, then the laser scan and visualization markers will need to be readded.
2. Select the laser scan and markers topics in the left-side pane and delete them.
3. Click "Add topic" at the bottom left. Go to "by topic", and select the laser scan topic.
4. Click "Add topic" at the bottom left. Go to "by topic", and select the visualization marker topic.
5. The LiDAR points and visualization markers should then be visible in RVIZ. If an error is thrown regarding the frames, or if the LiDAR and visualization markers still do not appear, check the following section.

If the LiDAR points and visualization markers are not appearing, then you may have to make some changes to the launch file.
1. Open the launch file with the editor of your choice.
2. Enter the topic name for your laser scan data in param name='scan_topic" value='/hsrb/base_scan". Replace /hsrb/base_scan with the name of your laser scan topic.
3. Enter the base frame being used by your LiDAR in param name="fixed_frame" value="base_link". Replace base_link with the base frame you are using.
4. Enter the desired frequency in param name="scan_frequency" value="10", replacing 10 with your desired value. This value is in Hertz, and has the possible options of 7.5Hz, 10Hz, and 15Hz.
5. If you do not have your laser scan topic name or base frame data, you can check it via the terminal. Open the terminal and type: rostopic list. Find the name of your laser scan topic within the list, often /scan or /scan_filtered on the Robile and /hsrb/base_scan for HSR. 
6. To find the base frame of the topic, enter: rostopic echo topic_name -n 1. Replace topic_name with the name of your scan topic. This will echo the first message from the topic. Scroll to the top of the message, where the header is. The frame_id paramter will show the name of the base frame for the scan topic. The minimum and maximum angles of the LiDAR view are also given here in radians (as well as the angle increment), and the minimum and maximum range of the LiDAR is given in meters. 

# Subscribed topics

/hsrb/base_scan: you can change the subscribed scan topic by changing the parameter "scan_topic" in the launch file.

/tf: to get the transforms. You should specify the fixed frame by the parameter "fixed_frame" in the launch file.

# Published topics

/visualization_marker : This publishes the visualization markers, which represent people detected from pairs of legs. The markers are mainly used by RVIZ. The markers share a base frame, given by frame_id, and each has a position given in meters with (x,y,z) coordinates. The orientation is always zero because the package cannot detect which way the person is facing from leg positions.

/people_tracked : This publishes the velocities and positions of the tracked people. Each person has a pose and orientation, as well as an ID number. The ID numbers can be used to track individual people. 

/detected_leg_clusters : This publishes all of the detected leg clusters. Each has an (x,y,z) position and a confidence. 

## Working:
1. Person will be detected using the leg pairs.
2. Robot will get the detetced person position and will follow the person.  

# Tunable parameters
To add a parameter to the launch file, use the following format: 

<param name="parameter_name" value="parameter_value" />

Replace parameter_name and parameter_value with the appropriate names and values. Useful parameters are described below.

## detect_leg_cluster.cpp

1. max_detected_clusters

The maximum number of clusters that will be tracked. It does closest to farthest, not cluster size. The default value is -1, for unlimited clusters.

2. forest_file

The path to the .yaml file containing the parameters for the random forest model. 

3. detection_threshold

The confidence threshold necessary to publish detected clusters. If the model does not have high confidence that the cluster is a leg, it will not be published. The default value is -1, meaning no threshold.

4. cluster_dist_euclid

The maximum distance between two points for them to be clustered together. The default value is 0.13 meters.

5. min_points_per_cluster

The minimum number of points necessary to create a cluster. The default value is 3.

6. max_detect_distance

The maximum distance at which clusters will be detected. Reducing this value improves run speed. The default value is 10 meters.

## joint_leg_tracker_follow.py

1. max_leg_pairing_dist

The maximum distance between a pair of legs for them to be considered as belonging to the same individual. The default value is 0.8 meters.

2. confidence_threshold_to_maintain_track

The confidence threshold determines how confident the model must be that a set of clusters are legs to begin and maintain a person track. The default value is 0.1, but it must be retuned if the leg detector is retrained.

3. dist_travelled_together_to_initiate_leg_pair

The distance a pair of legs must travel together to begin person tracking. The default value is 0.5 meters.

4. confidence_percentile

The confidence percentile used for matching clusters to person tracks. Increasing the percentile increases the distance with which tracks will match to new clusters. These distances are called gates, and are visible as circles around the person visualization markers in RVIZ. The default value is 0.6.

5. max_std

The maximum standard deviation of the covariance of a track. If the track's covariance exceeds this, the track is deleted. The default value is 0.9.
