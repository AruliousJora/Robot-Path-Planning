#! /usr/bin/env python

import rospy # Imports ROS functionalities
import numpy as np # To easily handle the arrays
import actionlib
# from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseFeedback, MoveBaseResult
from goal_publisher.msg import PointArray # To store the array of Points
from math import pi,radians,atan2,pow,sqrt # Perform mathematical operations so as to calculate the euclidean distance to goal and sterring angle to goal.
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from gazebo_msgs.msg import ModelStates # To obtain the position and orientation information of the robot.
from geometry_msgs.msg import Point # To declare a variable of Point structure with x=0,y=0,z=0
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan
from tf.transformations import euler_from_quaternion


class turtlebot:
    def __init__(self): # Constructor of class to initialise all the subscribers and publisher and the variables.
        number = rospy.get_param('/final_project/number')
        rospy.init_node('final_project') # ROS node initialisation
        rospy.Subscriber('/goals', PointArray, self.goals_callback) #'/robot'+str(number)+
        # rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.plan_callback)
        rospy.Subscriber('/robot'+str(number)+'/odom', Odometry, self.turtlebot_pose_callback)
        self.get_plan = rospy.ServiceProxy('/robot'+str(number)+'/move_base/make_plan', GetPlan)
        self.pub = rospy.Publisher('/robot'+str(number)+'/cmd_vel',Twist,queue_size=20)
        rospy.Subscriber('/robot'+str(number)+'/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/robot'+str(number)+'/scan', LaserScan, self.laser_callback)
        self.var = Twist()
        self.temp = [] # Shall store the initial list of 20 goals
        rospy.sleep(0.5)    # within this time, temp variable is updated in goals_callback function
        self.goal_arr = np.copy(self.temp)
        # Variable temp keeps updating with the value published in /goals topic,
        # but 'goal_arr' data member updates only once when created initially
        self.MB_client = actionlib.SimpleActionClient('/robot'+str(number)+'/move_base',MoveBaseAction)

        self.turtlebot_position = [] # Variable to save the position of bot
        self.skipped_goal = []
        self.linear_vel = 0
        self.ang_vel = 0

    def laser_callback(self, scan_msg): # Call back function for /scan topic
        self.laser_msg = scan_msg
        self.robfront =  min(min(self.laser_msg.ranges[0:15]),min(self.laser_msg.ranges[345:360]), 7)
        self.robfleft =  min(min(self.laser_msg.ranges[16:25]), 7)
        self.robfright =  min(min(self.laser_msg.ranges[335:345]), 7)

    def plan_callback(self,plan_msg):
        self.waypoints_temp = []
        all_pose_stamped = plan_msg.poses
        for pose_stamped in all_pose_stamped:
            pose = pose_stamped.pose.position
            self.waypoints_temp.append(pose)

    def cmd_vel_callback(self,velmsg):
        self.linear_vel = velmsg.linear.x
        self.ang_vel = velmsg.angular.z


    def distance(self, p1,p2):  # Calculate distance between two points.

        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y))

    def goals_callback(self, goals_msg):  # Call back function for /goals topic
        self.temp = goals_msg.goals
        self.num_of_goals = len(self.temp)
        # print self.temp


    def turtlebot_pose_callback(self, pose_msg):
        turtlebot_pose = pose_msg.pose.pose
        self.turtlebot_position = turtlebot_pose.position

        turtlebot_orientation = turtlebot_pose.orientation
        turtlebot_orientation_list = [turtlebot_orientation.x, turtlebot_orientation.y, turtlebot_orientation.z, turtlebot_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (turtlebot_orientation_list)
        self.turtlebot_yaw_angle = yaw * 180 / pi

    def get_next_goal(self, array_goals):   # To get the next nearest goal from after the bot reaches the current goal.
        temp_goals=[]
        for i in range(len(array_goals)):
            temp_goals.append(array_goals[i])
        # temp_goals.sort( key=lambda x: self.distance(self.turtlebot_position, x))
        temp_goals.sort( key=lambda x: x.reward)
        # Sorting is performed based on distance to all goals from robot's current position in ascending order.
        goal = temp_goals[0]

        return goal


    def init_condition(self):   # To stop the bot.

    	self.var.linear.x=0
    	self.var.angular.z=0
    	self.pub.publish(self.var)
    	rospy.sleep(0.2)

    def linear_move(self):  # To move the bot linearly.

        self.var.linear.x =0.11
        self.var.angular.z = 0
        self.pub.publish(self.var)

    def steering_angle(self,p1,p2): # Calculate slope of line joining two points
        ang = atan2(p2.y - p1.y, p2.x - p1.x)
        ang = ang * 180 / pi # This the yaw angle that the robot should reach.

        return ang

    def rob_turn(self, angle):  # Turn the bot to a specific angle.

        while(1):

            ang_diff = angle-self.turtlebot_yaw_angle
            if ang_diff < 0:
                self.var.angular.z=0.25 * -1
            if ang_diff > 0:
                self.var.angular.z=0.25 * +1
            self.var.linear.x = 0
            self.pub.publish(self.var)

            if(abs(ang_diff) < 0.1):
                break

    def service_plan_call(self,goal):
        self.waypoints_temp = []
        start = PoseStamped()
        start.header.seq = 0
        start.header.frame_id = "map"
        start.header.stamp = rospy.Time(0)
        start.pose.position.x = self.turtlebot_position.x #2.6
        start.pose.position.y = self.turtlebot_position.y  #1.3

        Goal = PoseStamped()
        Goal.header.seq = 0
        Goal.header.frame_id = "map"
        Goal.header.stamp = rospy.Time(0)
        Goal.pose.position.x = goal.x  #-6.2
        Goal.pose.position.y = goal.y  #-3.0

        req = GetPlan()
        req.start = start
        req.goal = Goal
        req.tolerance = 0.5
        resp = self.get_plan(req.start, req.goal, req.tolerance)
        all_pose_stamped = resp.plan.poses
        for pose_stamped in all_pose_stamped:
            pose = pose_stamped.pose.position
            self.waypoints_temp.append(pose)


    def distance_move(self,distance):
        p1 = self.turtlebot_position
        while 1:
            self.linear_move()
            p2 = self.turtlebot_position
            if self.distance(p1,p2)> distance :
                break



    def move_bot(self):
        obs_dist = 0.3
        while(len(self.goal_arr) != 0): # To reach the goals in goal_arr and then exit without error
            print "now next goal"
            # print self.MB_client.get_state()
            goal = self.get_next_goal(self.goal_arr)
            self.movebase_client(goal)

            print goal

            t1 = rospy.get_time()
            # t1_a = rospy.get_time()
            self.dist_2 = 0

            while True:
                # print self.MB_client.get_state()
                self.dist = self.distance(goal, self.turtlebot_position)    # keep track of distance to the goal
                t2 = rospy.get_time()
                #
                # t2_a = rospy.get_time()
                if(self.MB_client.get_state() == 3 or self.dist < 0.1 ):
                    print "Reached Goal: ", goal   # Distance to goal 0.3, considered as reached.
                    break
                if (t2-t1) > 70:

                    self.MB_client.cancel_goal()
                    self.skipped_goal.append(goal)
                    self.movebase_client(Point())
                    t1 = rospy.get_time()
                    self.MB_client.wait_for_result()
                    if self.MB_client.get_state() == 3:
                        print "reached origin"

                    print "Aborted goal"
                    break


                if self.MB_client.get_state() == 4 or (self.linear_vel == 0.0 and self.ang_vel == 0.0 and (t2-t1) > 15):
                    rospy.sleep(3)
                    if self.linear_vel == 0.0 and self.ang_vel == 0.0:
                        self.MB_client.cancel_goal()
                        rospy.sleep(1)
                        self.init_condition()

                        self.service_plan_call(goal)
                        self.waypoints = np.copy(self.waypoints_temp)

                        try:
                            self.rob_turn(self.steering_angle(self.turtlebot_position,self.waypoints[5]))
                            self.init_condition()
                            while 1:
                                if self.distance(self.waypoints[5], self.turtlebot_position) < 0.05:
                                    break

                                if self.robfront > obs_dist:
                                    self.linear_move()
                                else:
                                    if self.robfront <= obs_dist:

                                        if self.robfleft >= 1:   #  Check if no obstacle on left side

                                            self.init_condition()
                                            while (self.robfront <= obs_dist):


                                                self.var.angular.z=0.11 * +1 # Positive acceleration to move anticlockwise
                                                self.var.linear.x = 0
                                                self.pub.publish(self.var)


                                        elif self.robfright >= 1:    # Check if no obstacle on right side

                                            self.init_condition()
                                            while (self.robfront <= obs_dist):


                                                self.var.angular.z=0.11 * -1 # Negative acceleration to move clockwise (rightwards)
                                                self.var.linear.x = 0
                                                self.pub.publish(self.var)

                                        self.distance_move(0.5)
                                        self.init_condition()
                                        self.rob_turn(self.steering_angle(self.turtlebot_position,self.waypoints[5]))
                        except:
                            a=0


                        self.movebase_client(goal)
                        t1 = rospy.get_time()

                        continue



            self.prev_goal = goal
            self.goal_arr = [x for x in self.goal_arr if x != goal] # After reaching the goal remove it from the array
        print self.skipped_goal

    def movebase_client(self,goal):

        self.MB_client.wait_for_server() # The function waits until the server is ready to receive goals.

        MB_goal = MoveBaseGoal() # Ref:  http://docs.ros.org/en/fuerte/api/move_base_msgs/html/msg/MoveBaseGoal.html
        MB_goal.target_pose.header.frame_id = "map"
        MB_goal.target_pose.header.stamp = rospy.Time.now()
        MB_goal.target_pose.pose.position.x = goal.x
        MB_goal.target_pose.pose.position.y = goal.y
        MB_goal.target_pose.pose.orientation.w = 1.0


        self.MB_client.send_goal(MB_goal)



if __name__ == '__main__':

    t = turtlebot() # Instance of class created and so constructor function of class is called.
    rospy.sleep(0.2)
    t.move_bot() # To start the path planning to reach the goals.
    t.goal_arr = t.skipped_goal
    t.move_bot()
    while not rospy.is_shutdown(): # After reachng all the goals, control waits here until the user stops the node using ctrl+c
        rospy.sleep(0.5)
