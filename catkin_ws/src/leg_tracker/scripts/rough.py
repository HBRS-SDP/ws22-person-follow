
#----------------
import controller_manager_msgs.srv
#----------------

# from hsrb_interface import Robot

# import actionlib
# from actionlib_msgs.msg import GoalStatus
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


    initial_iter = True
    temp_x, temp_y, temp_yaw = 0, 0, 0
    rotate_angle = 30 # degrees
    init_rotate = -60 # degrees
    counter_rotate = 0
    dir_rot = True



        # Robot current postition
        


break_true = True
        temp_x, temp_y, temp_yaw = 0, 0, 0
        temp_quart = [0, 0, 0, 0]



temp_x = new_person.pose.position.x
                    temp_y = new_person.pose.position.y
                    temp_yaw = yaw
                    temp_quart = quaternion

                    # self.people_tracked_pub.publish()
                    # print('people tr pub : ',self.people_tracked_pub)
                    self.move(new_person.pose.position.x,new_person.pose.position.y,yaw,yaw2)
                    # print('move_x: {}\nmove_y: {}\nmove_yaw: {}\nyaw (rad) : {}'.format(temp_x,temp_y,temp_yaw, np.rad2deg(temp_yaw)))
                    # break_true = False

                    
                # if KalmanMultiTracker.initial_iter != True and break_true == True:
                #     if KalmanMultiTracker.counter_rotate >= 0 and KalmanMultiTracker.dir_rot == True:
                #         if KalmanMultiTracker.counter_rotate == 0:
                #             KalmanMultiTracker.counter_rotate += 1
                #             self.outOfFocus(now, np.deg2rad(KalmanMultiTracker.init_rotate))
                #         elif KalmanMultiTracker.counter_rotate<5:
                #             KalmanMultiTracker.counter_rotate += 1
                #             self.outOfFocus(now, np.deg2rad(KalmanMultiTracker.rotate_angle))
                #         else:
                #             KalmanMultiTracker.dir_rot = False
                #     elif KalmanMultiTracker.dir_rot == False:
                #         if KalmanMultiTracker.counter_rotate > 0:
                #             KalmanMultiTracker.counter_rotate -= 1
                #             self.outOfFocus(now, np.deg2rad(- KalmanMultiTracker.rotate_angle))
                #         elif KalmanMultiTracker.counter_rotate == 0:
                #             KalmanMultiTracker.dir_rot = True
                # # print('Counter : ', KalmanMultiTracker.counter_rotate)



   
    def robot_pose_callback(self,msg):
        self.msg_odom = msg
        self.x_robot = msg.pose.pose.position.x
        self.y_robot = msg.pose.pose.position.y
        self.quat_robot = msg.pose.pose.orientation
        orientation_list = [self.quat_robot.x, self.quat_robot.y, self.quat_robot.z, self.quat_robot.w]
        
        _, _, self.yaw_robot = tf.transformations.euler_from_quaternion(orientation_list)

    def outOfFocus(self, now, yaw):
        # pub = rospy.Publisher('goal',PoseStamped,queue_size=10)
        while self.pub.get_num_connections()<2:
            rospy.sleep(0.1)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'base_link'
        quat = tf.transformations.quaternion_from_euler(0,0,yaw)
        goal.pose.orientation = Quaternion(*quat)
        goal.pose.position = Point(0,0,0)
        self.pub.publish(goal)
        print(' Angle : ',np.rad2deg(yaw))
        rospy.sleep(5.0)
        print('Sleep completed --- to allow robot to complete rotation')
        self.publish_tracked_people(now)
        

    def move(self, x, y, yaw, yaw2): 
        x = x
        y = y
        yaw = yaw

        thresh = 0.1
        k_p = 0.4

        print('Robot position: ',self.x_robot, self.y_robot, self.yaw_robot)

        x_err = x - self.x_robot - thresh
        y_err = y - self.y_robot - thresh

        print('x-err : {} | y-err : {}'.format(x_err,y_err))

        if round(x,2) - round(self.x_robot,2) == 0:
            x_err = 0
            print('x_err reset /////////////////----------------')

        if round(y,2) - round(self.y_robot,2) == 0:
            y_err = 0
            print('y_err reset /////////////////----------------')

        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'omni_base_controller' and c.state == 'running':
                    running = True
        
        tw = geometry_msgs.msg.Twist()

        tw.linear.x = x_err * k_p
        tw.linear.y = y_err * k_p

        self.pub.publish(tw)

        
        print('------------------------- PEOPLE TRACKED : {} --------------------------'.format(len(self.people_tracked)))
        print('move_x: {}\nmove_y: {}\nmove_yaw: {}\nRobot Moved....xxxxxxxxxxxxxxxxx.....................'.format(x,y,yaw))
    











    # def move(self, x, y, yaw, yaw2): # old goal & command velocity with conditions
    #     x = x
    #     y = y
    #     yaw = yaw
    #     yaw2 = yaw2
    #     print('move_x : {}\nmove_y : {}\nmove_yaw : {}'.format(x,y,yaw))
    #     # pub = rospy.Publisher('goal',PoseStamped,queue_size=10)
    #     while self.pub.get_num_connections()<2:
    #         rospy.sleep(0.1)
        
    #     goal = PoseStamped()
    #     goal.header.stamp = rospy.Time.now()
    #     goal.header.frame_id = 'base_link'


    #     rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
    #     list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
    #     running = False
    #     while running is False:
    #         rospy.sleep(0.1)
    #         for c in list_controllers().controller:
    #             if c.name == 'omni_base_controller' and c.state == 'running':
    #                 running = True
        
    #     tw = geometry_msgs.msg.Twist()
    #     # goal_yaw = 0      

    #     if KalmanMultiTracker.initial_iter == True:
    #         print('In KalmanMultiTracker.initial_iter = True loop ')
    #         if y < 0:
    #             if x != 0 :
    #                 goal_yaw = min(yaw, yaw2)
    #                 print('y<0 Min of Yaw taken here : ', np.rad2deg(goal_yaw))
    #             else:
    #                 goal_yaw = 0
    #                 print(' y<0 but x==0 : ',goal_yaw)
    #         elif y > 0:
    #             if x != 0 :
    #                 goal_yaw = max(yaw, yaw2)
    #                 print('y<0 Max of Yaw taken here : ', np.rad2deg(goal_yaw))
    #             else:
    #                 goal_yaw = 0

    #                 print(' y>0 but x==0 : ',goal_yaw)
    #     else:
    #         print('In KalMan false loop')
    #         if y < 0:
    #             if x != 0 :
    #                 goal_yaw = min(yaw, yaw2)
    #                 print('y>0 Min of Yaw taken here : ', np.rad2deg(goal_yaw))
    #             else:
    #                 goal_yaw = 0
    #                 print(' y<0 but x==0 : ',goal_yaw)
    #         elif y > 0:
    #             if x != 0 :
    #                 goal_yaw = max(yaw, yaw2)
    #                 print('y>0 Max of Yaw taken here : ', np.rad2deg(goal_yaw))
    #             else:
    #                 goal_yaw = 0

    #                 print(' y>0 but x==0 : ',goal_yaw)
    #         if goal_yaw <np.deg2rad(0.5) and goal_yaw > np.deg2rad(-0.5): # 5.0
    #             goal_yaw = 0
    #             print('Goal_yaw is between 0.5 & -0.5')

    
    #     KalmanMultiTracker.initial_iter = False

        
    #     g = 0.05
    #     if KalmanMultiTracker.temp_x+g >= x and KalmanMultiTracker.temp_x-g <= x:
    #         print('New loop')
    #         goal_yaw = 0
    #     else:
    #         print('New else loop')
    #         KalmanMultiTracker.temp_x = x
    #         KalmanMultiTracker.temp_y = y
    #     KalmanMultiTracker.temp_yaw = goal_yaw

    #     quat = tf.transformations.quaternion_from_euler(0,0,goal_yaw)
    #     goal.pose.orientation = Quaternion(*quat)

    #     print('\nKalmanMultiTracker.temp_x : ',KalmanMultiTracker.temp_x)
    #     print('KalmanMultiTracker.temp_y : ',KalmanMultiTracker.temp_y)
    #     print('KalmanMultiTracker.temp_yaw : ',np.rad2deg(KalmanMultiTracker.temp_yaw))

    #     try:
    #         x = x - 0.05
    #         y = y

    #         tw.linear.x =  0.08 #person_vel_x
    #         tw.linear.y = 0

    #         goal.pose.position = Point(x,y,0)
            
    #         self.pub.publish(goal)
    #         self.pub0.publish(tw)
            
    #         print('------------------------- PEOPLE TRACKED : {} --------------------------'.format(len(self.people_tracked)))
    #         print('move_x: {}\nmove_y: {}\nmove_yaw: {}\nRobot Moved....xxxxxxxxxxxxxxxxx.....................'.format(x,y,yaw))
    #     except KeyboardInterrupt:
    #         exit()


    # def move(self,now):  # with Marker IDs published and yaw is calculated and action client is used within this function
    #     x = 0
    #     y = 0
    #     yaw = 0
    #     quaternion = [0, 0, 0, 0]

    #     pub = rospy.Publisher('goal',PoseStamped,queue_size=10)
    #     while pub.get_num_connections()<2:
    #         rospy.sleep(0.1)

    #     goal = PoseStamped()
    #     goal.header.stamp = rospy.Time.now()
    #     goal.header.frame_id = 'base_link'

    #     people_tracked_msg = PersonArray()
    #     people_tracked_msg.header.stamp = now
    #     people_tracked_msg.header.frame_id = self.publish_people_frame        
    #     marker_id = 0
    #     print('people_tracked_msg.header.stamp : ',people_tracked_msg.header.stamp)

    #     # Make sure we can get the required transform first:
    #     if self.use_scan_header_stamp_for_tfs:
    #         tf_time = now
    #         try:
    #             print('Try')
    #             self.listener.waitForTransform(self.publish_people_frame, self.fixed_frame, tf_time, rospy.Duration(1.0))
    #             transform_available = True
    #         except Exception as e:
    #             print('Exception at line 647 : ',str(e))
    #             transform_available = False
    #     else:
    #         tf_time = rospy.Time(0)
    #         transform_available = self.listener.canTransform(self.publish_people_frame, self.fixed_frame, tf_time)

    #     marker_id = 0
    #     if not transform_available:
    #         rospy.loginfo("Person tracker: tf not avaiable. Not publishing people")
    #     else:
    #         for person in self.objects_tracked:
    #             if person.is_person == True:
    #                 if self.publish_occluded or person.seen_in_current_scan: # Only publish people who have been seen in current scan, unless we want to publish occluded people
    #                     ps = PointStamped()
    #                     ps.header.frame_id = self.fixed_frame
    #                     ps.header.stamp = tf_time
    #                     ps.point.x = person.pos_x
    #                     ps.point.y = person.pos_y
    #                     try:
    #                         ps = self.listener.transformPoint(self.publish_people_frame, ps)
    #                     except Exception as e:
    #                         print('Exception : ',str(e))
    #                         rospy.logerr("Not publishing people due to no transform from fixed_frame-->publish_people_frame")                                                
    #                         continue
                        
    #                     new_person = Person() 
    #                     new_person.pose.position.x = ps.point.x 
    #                     new_person.pose.position.y = ps.point.y 
    #                     yaw = math.atan2(person.vel_y, person.vel_x)
    #                     yaw2 = math.atan2(new_person.pose.position.y, new_person.pose.position.x)
    #                     print('Yaw from person position & velocity : ', np.rad2deg(yaw2), np.rad2deg(yaw))
    #                     # quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    #                     # print('\nQuar_x: {}\nQuat_y:{}\nQuat_z:{}\nQuat_w:{}\n\n'.format(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
    #                     # new_person.pose.orientation.x = quaternion[0]
    #                     # new_person.pose.orientation.y = quaternion[1]
    #                     # new_person.pose.orientation.z = quaternion[2]
    #                     # new_person.pose.orientation.w = quaternion[3] 
    #                     new_person.id = person.id_num 
    #                     print('New_person_id :',new_person.id)
    #                     people_tracked_msg.people.append(new_person)
    #                     print('People_tracked_msg : ',people_tracked_msg.people)

    #                     # publish rviz markers       
    #                     # Cylinder for body 
    #                     marker = Marker()
    #                     marker.header.frame_id = self.publish_people_frame
    #                     marker.header.stamp = now
    #                     marker.ns = "People_tracked"
    #                     marker.color.r = person.colour[0]
    #                     marker.color.g = person.colour[1]
    #                     marker.color.b = person.colour[2]          
    #                     marker.color.a = (rospy.Duration(3) - (rospy.get_rostime() - person.last_seen)).to_sec()/rospy.Duration(3).to_sec() + 0.1
    #                     marker.pose.position.x = ps.point.x 
    #                     marker.pose.position.y = ps.point.y
    #                     marker.id = marker_id
    #                     print('Person marker.id : ',marker.id)
    #                     marker_id += 1
    #                     marker.type = Marker.CYLINDER
    #                     marker.scale.x = 0.2
    #                     marker.scale.y = 0.2
    #                     marker.scale.z = 1.2
    #                     marker.pose.position.z = 0.8
    #                     self.marker_pub.publish(marker)  
    #                     # Sphere for head shape                        
    #                     marker.type = Marker.SPHERE
    #                     marker.scale.x = 0.2
    #                     marker.scale.y = 0.2
    #                     marker.scale.z = 0.2                
    #                     marker.pose.position.z = 1.5
    #                     marker.id = marker_id 
    #                     marker_id += 1                        
    #                     self.marker_pub.publish(marker)     

    #                     # Text showing person's ID number 
    #                     marker.color.r = 1.0
    #                     marker.color.g = 1.0
    #                     marker.color.b = 1.0
    #                     marker.color.a = 1.0
    #                     marker.id = marker_id
    #                     marker_id += 1
    #                     marker.type = Marker.TEXT_VIEW_FACING
    #                     marker.text = str(person.id_num)
    #                     print('Marker.text : ',marker.text)
    #                     marker.scale.z = 0.2         
    #                     marker.pose.position.z = 1.7
    #                     self.marker_pub.publish(marker)

    #                     # Arrow pointing in direction they're facing with magnitude proportional to speed
    #                     marker.color.r = person.colour[0]
    #                     marker.color.g = person.colour[1]
    #                     marker.color.b = person.colour[2]          
    #                     marker.color.a = (rospy.Duration(3) - (rospy.get_rostime() - person.last_seen)).to_sec()/rospy.Duration(3).to_sec() + 0.1                        
    #                     start_point = Point()
    #                     end_point = Point()
    #                     start_point.x = marker.pose.position.x 
    #                     start_point.y = marker.pose.position.y 
    #                     end_point.x = start_point.x + 0.5*person.vel_x
    #                     end_point.y = start_point.y + 0.5*person.vel_y
    #                     marker.pose.position.x = 0.
    #                     marker.pose.position.y = 0.
    #                     marker.pose.position.z = 0.1
    #                     marker.id = marker_id
    #                     marker_id += 1
    #                     marker.type = Marker.ARROW
    #                     marker.points.append(start_point)
    #                     marker.points.append(end_point)
    #                     marker.scale.x = 0.4
    #                     marker.scale.y = 0.4
    #                     marker.scale.z = 0.4
    #                     self.marker_pub.publish(marker)                           

    #                     # <self.confidence_percentile>% confidence bounds of person's position as an ellipse:
    #                     cov = person.filtered_state_covariances[0][0] + person.var_obs # cov_xx == cov_yy == cov
    #                     std = cov**(1./2.)
    #                     gate_dist_euclid = scipy.stats.norm.ppf(1.0 - (1.0-self.confidence_percentile)/2., 0, std)
    #                     marker.pose.position.x = ps.point.x 
    #                     marker.pose.position.y = ps.point.y                    
    #                     marker.type = Marker.SPHERE
    #                     marker.scale.x = 2*gate_dist_euclid
    #                     marker.scale.y = 2*gate_dist_euclid
    #                     marker.scale.z = 0.01   
    #                     marker.color.r = person.colour[0]
    #                     marker.color.g = person.colour[1]
    #                     marker.color.b = person.colour[2]            
    #                     marker.color.a = 0.1
    #                     marker.pose.position.z = 0.0
    #                     marker.id = marker_id 
    #                     marker_id += 1                    
    #                     self.marker_pub.publish(marker) 
                    
    #                 if KalmanMultiTracker.initial_iter == True:
    #                     print('In KalmanMultiTracker.initial_iter = True loop ')
    #                     if new_person.pose.position.y < 0:
    #                         if new_person.pose.position.x != 0 :
    #                             goal_yaw = min(yaw, yaw2)
    #                             print('y<0 Min of Yaw taken here : ', np.rad2deg(goal_yaw))
    #                         else:
    #                             goal_yaw = 0
    #                             print(' y<0 but x==0 : ',goal_yaw)
    #                     elif new_person.pose.position.y > 0:
    #                         if new_person.pose.position.x != 0 :
    #                             goal_yaw = max(yaw, yaw2)
    #                             print('y<0 Max of Yaw taken here : ', np.rad2deg(goal_yaw))
    #                         else:
    #                             goal_yaw = 0

    #                             print(' y>0 but x==0 : ',goal_yaw)
    #                 else:
    #                     print('In KalMan false loop')
    #                     if new_person.pose.position.y < 0:
    #                         if new_person.pose.position.x != 0 :
    #                             goal_yaw = min(yaw, yaw2)
    #                             print('y>0 Min of Yaw taken here : ', np.rad2deg(goal_yaw))
    #                         else:
    #                             goal_yaw = 0
    #                             print(' y<0 but x==0 : ',goal_yaw)
    #                     elif new_person.pose.position.y > 0:
    #                         if new_person.pose.position.x != 0 :
    #                             goal_yaw = max(yaw, yaw2)
    #                             print('y>0 Max of Yaw taken here : ', np.rad2deg(goal_yaw))
    #                         else:
    #                             goal_yaw = 0

    #                             print(' y>0 but x==0 : ',goal_yaw)
    #                     if goal_yaw <np.deg2rad(0.5) and goal_yaw > np.deg2rad(-0.5): # 5.0
    #                         goal_yaw = 0
    #                         print('Goal_yaw is between 5 & -5')

                
    #                 KalmanMultiTracker.initial_iter = False

                    
    #                 g = 0.05
    #                 if KalmanMultiTracker.temp_x+g >= new_person.pose.position.x and KalmanMultiTracker.temp_x-g <= new_person.pose.position.x:
    #                     print('New loop')
    #                     goal_yaw = 0
    #                 else:
    #                     print('New else loop')
    #                     KalmanMultiTracker.temp_x = new_person.pose.position.x
    #                     KalmanMultiTracker.temp_y = new_person.pose.position.y
    #                 KalmanMultiTracker.temp_yaw = goal_yaw

    #                 quat = tf.transformations.quaternion_from_euler(0,0,goal_yaw)
    #                 goal.pose.orientation = Quaternion(*quat)

    #                 print('\nKalmanMultiTracker.temp_x : ',KalmanMultiTracker.temp_x)
    #                 print('KalmanMultiTracker.temp_y : ',KalmanMultiTracker.temp_y)
    #                 print('KalmanMultiTracker.temp_yaw : ',np.rad2deg(KalmanMultiTracker.temp_yaw))

    #                 try:
    #                     goal_x = new_person.pose.position.x - 0.2
    #                     goal_y = new_person.pose.position.y

    #                     goal.pose.position = Point(goal_x, goal_y, 0)

    #                     pub.publish(goal)
                        
    #                     print('------------------------- PEOPLE TRACKED : {} --------------------------'.format(len(self.people_tracked)))
    #                     print('move_x: {}\nmove_y: {}\nmove_yaw: {}\nmove_qaurternion:{}\nRobot Moved....xxxxxxxxxxxxxxxxx.....................'.format(x,y,yaw, quaternion))
    #                 except KeyboardInterrupt:
    #                     exit()
    #     # Publish people tracked message
    #     self.people_tracked_pub.publish()
    #     print('people_tracked_pub published...')  
        

    # def move(self, now): # with action client & markers are published and yaw is calculated in this function
    #     # initialize action client
    #     cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        
    #     people_tracked_msg = PersonArray()
    #     people_tracked_msg.header.stamp = now
    #     people_tracked_msg.header.frame_id = self.publish_people_frame        
    #     marker_id = 0
    #     print('people_tracked_msg.header.stamp : ',people_tracked_msg.header.stamp)

    #     # Make sure we can get the required transform first:
    #     if self.use_scan_header_stamp_for_tfs:
    #         tf_time = now
    #         try:
    #             print('Try')
    #             self.listener.waitForTransform(self.publish_people_frame, self.fixed_frame, tf_time, rospy.Duration(1.0))
    #             transform_available = True
    #         except Exception as e:
    #             print('Exception at line 647 : ',str(e))
    #             transform_available = False
    #     else:
    #         tf_time = rospy.Time(0)
    #         transform_available = self.listener.canTransform(self.publish_people_frame, self.fixed_frame, tf_time)

    #     marker_id = 0
    #     if not transform_available:
    #         rospy.loginfo("Person tracker: tf not avaiable. Not publishing people")
    #     else:
    #         for person in self.objects_tracked:
    #             print('Person : ', person.pos_x, person.pos_y)
    #             if person.is_person == True:
    #                 if self.publish_occluded or person.seen_in_current_scan: # Only publish people who have been seen in current scan, unless we want to publish occluded people
    #                     ps = PointStamped()
    #                     ps.header.frame_id = self.fixed_frame
    #                     ps.header.stamp = tf_time
    #                     ps.point.x = person.pos_x
    #                     ps.point.y = person.pos_y
    #                     try:
    #                         ps = self.listener.transformPoint(self.publish_people_frame, ps)
    #                     except Exception as e:
    #                         print('Exception : ',str(e))
    #                         rospy.logerr("Not publishing people due to no transform from fixed_frame-->publish_people_frame")                                                
    #                         continue
                        
    #                     new_person = Person() 
    #                     new_person.pose.position.x = ps.point.x 
    #                     new_person.pose.position.y = ps.point.y 
    #                     yaw = math.atan2(person.vel_y, person.vel_x)
    #                     yaw2 = math.atan2(new_person.pose.position.y, new_person.pose.position.x)
    #                     print('Yaw from person position & velocity : ', np.rad2deg(yaw2), np.rad2deg(yaw))
    #                     quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    #                     print('\nQuar_x: {}\nQuat_y:{}\nQuat_z:{}\nQuat_w:{}\n\n'.format(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
    #                     new_person.pose.orientation.x = quaternion[0]
    #                     new_person.pose.orientation.y = quaternion[1]
    #                     new_person.pose.orientation.z = quaternion[2]
    #                     new_person.pose.orientation.w = quaternion[3] 
    #                     new_person.id = person.id_num 
    #                     print('New_person_id :',new_person.id)
    #                     people_tracked_msg.people.append(new_person)
    #                     print('People_tracked_msg : ',people_tracked_msg.people)
                    
    #                 if KalmanMultiTracker.initial_iter == True:
    #                     print('In KalmanMultiTracker.initial_iter = True loop ')
    #                     if new_person.pose.position.y < 0:
    #                         if new_person.pose.position.x != 0 :
    #                             goal_yaw = min(yaw, yaw2)
    #                             print('y<0 Min of Yaw taken here : ', np.rad2deg(goal_yaw))
    #                         else:
    #                             goal_yaw = 0
    #                             print(' y<0 but x==0 : ',goal_yaw)
    #                     elif new_person.pose.position.y > 0:
    #                         if new_person.pose.position.x != 0 :
    #                             goal_yaw = max(yaw, yaw2)
    #                             print('y<0 Max of Yaw taken here : ', np.rad2deg(goal_yaw))
    #                         else:
    #                             goal_yaw = 0

    #                             print(' y>0 but x==0 : ',goal_yaw)
    #                 else:
    #                     print('In KalMan false loop')
    #                     if new_person.pose.position.y < 0:
    #                         if new_person.pose.position.x != 0 :
    #                             goal_yaw = min(yaw, yaw2)
    #                             print('y>0 Min of Yaw taken here : ', np.rad2deg(goal_yaw))
    #                         else:
    #                             goal_yaw = 0
    #                             print(' y<0 but x==0 : ',goal_yaw)
    #                     elif new_person.pose.position.y > 0:
    #                         if new_person.pose.position.x != 0 :
    #                             goal_yaw = max(yaw, yaw2)
    #                             print('y>0 Max of Yaw taken here : ', np.rad2deg(goal_yaw))
    #                         else:
    #                             goal_yaw = 0

    #                             print(' y>0 but x==0 : ',goal_yaw)
    #                     if goal_yaw <np.deg2rad(0.5) and goal_yaw > np.deg2rad(-0.5): # 5.0
    #                         goal_yaw = 0
    #                         print('Goal_yaw is between 5 & -5')

                
    #                 KalmanMultiTracker.initial_iter = False

                    
    #                 g = 0.05
    #                 if KalmanMultiTracker.temp_x+g >= new_person.pose.position.x and KalmanMultiTracker.temp_x-g <= new_person.pose.position.x:
    #                     print('New loop')
    #                     goal_yaw = 0
    #                 else:
    #                     print('New else loop')
    #                     KalmanMultiTracker.temp_x = new_person.pose.position.x
    #                     KalmanMultiTracker.temp_y = new_person.pose.position.y
    #                 KalmanMultiTracker.temp_yaw = goal_yaw

    #                 print('\nKalmanMultiTracker.temp_x : ',KalmanMultiTracker.temp_x)
    #                 print('KalmanMultiTracker.temp_y : ',KalmanMultiTracker.temp_y)
    #                 print('KalmanMultiTracker.temp_yaw : ',np.rad2deg(KalmanMultiTracker.temp_yaw))
    #                 print()
    #     # print('people_tracked_msg.header.stamp : ',people_tracked_msg.header.stamp)

    #                 try:
    #                     # wait for the action server to establish connection
    #                     # cli.wait_for_server()

    #                     # input goal pose
    #                     goal_x = new_person.pose.position.x - 0.2
    #                     goal_y = new_person.pose.position.y
    #                     # goal_yaw = yaw

    #                     # fill ROS message
    #                     pose = PoseStamped()
    #                     pose.header.stamp = rospy.Time.now()
    #                     pose.header.frame_id = "odom"
    #                     pose.pose.position = Point(ps.point.x, ps.point.y, 0)
    #                     print('quat : ',np.rad2deg(KalmanMultiTracker.temp_yaw))
    #                     quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
    #                     pose.pose.orientation = Quaternion(*quat)
    #                     # pose.pose.orientation = Quaternion(*quaternion)

    #                     goal = MoveBaseGoal()
    #                     goal.target_pose = pose
    #                     print('Target Pose : ', goal.target_pose)

    #                     # send message to the action server
    #                     cli.send_goal(goal)
    #                     print(' Goal sent ... waiting for result ...')

    #                     # wait for the action server to complete the order
    #                     cli.wait_for_result()
    #                     # rospy.sleep(1.0)
    #                     print(' Done waiting ... ')
    #                     quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    #                     pose.pose.orientation = Quaternion(*quat)
    #                     pose.pose.position = Point(0, 0, 0)
    #                     goal.target_pose = pose
    #                     print('Target Pose : ', goal.target_pose)

    #                     # send message to the action server
    #                     cli.send_goal(goal)
    #                     print('sleep')
    #                     # rospy.sleep(5.0)
    #                     rospy.loginfo("Navigation Succeeded.")
    #                     # print result of navigation
    #                     action_state = cli.get_state()
    #                     if action_state == GoalStatus.SUCCEEDED:
    #                         quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    #                         pose.pose.orientation = Quaternion(*quat)
    #                         pose.pose.position = Point(0, 0, 0)
    #                         goal.target_pose = pose
    #                         print('Target Pose : ', goal.target_pose)

    #                         # send message to the action server
    #                         cli.send_goal(goal)
    #                         print('sleep')
    #                         # rospy.sleep(5.0)
    #                         rospy.loginfo("Navigation Succeeded.")
    #                 except KeyboardInterrupt:
    #                     exit()
    #                 # rospy.sleep(1.0)
    # # Publish people tracked message
    #     self.people_tracked_pub.publish()
    #     print('people_tracked_pub published...') 



    
    # def move1(self, x, y, yaw, quaternion): # with only the action client

    #     try:
    #         # wait for the action server to establish connection
    #         cli.wait_for_server()

    #         # input goal pose
    #         goal_x = x
    #         goal_y = y
    #         goal_yaw = yaw

    #         # fill ROS message
    #         pose = PoseStamped()
    #         pose.header.stamp = rospy.Time.now()
    #         pose.header.frame_id = "base_link"
    #         pose.pose.position = Point(goal_x, goal_y, 0)
    #         quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
    #         pose.pose.orientation = Quaternion(*quat)

    #         goal = MoveBaseGoal()
    #         goal.target_pose = pose

    #         # send message to the action server
    #         cli.send_goal(goal)
    #         print(' Goal sent ... waiting for result ...')

    #         # wait for the action server to complete the order
    #         cli.wait_for_result()
    #         print(' Result received ... ')
    #         # print result of navigation
    #         action_state = cli.get_state()
    #         if action_state == GoalStatus.SUCCEEDED:
    #             rospy.loginfo("Navigation Succeeded.")
    #     except KeyboardInterrupt:
    #         exit()
 