#!/usr/bin/env python3

"""
This is a node called turtle_driver - server

"""

import turtle
import rospy
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from turtle_driver.srv import DriveTurtleSrv

class TurtleBot:

    def __init__(self, radius, side_length, waypoints):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        # rospy.init_node('turtlebot_controller', anonymous=True)

        
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                              Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.radius = radius
        self.side_length = side_length
        self.waypoints = waypoints
        self.original_pose = Pose()
        self.original_pose.x = 5.5444
        self.original_pose.y = 5.5444
        self.square_corners = []


    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)


    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return math.sqrt(math.pow((goal_pose.x - self.pose.x), 2) +
                    math.pow((goal_pose.y - self.pose.y), 2))


    def steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)


    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)


    def angular_vel(self, goal_pose, constant=6):
        goal_theta = self.steering_angle(goal_pose)
        return constant * clamp(goal_theta - self.pose.theta)


    def move2goal(self, goal_pose):
        """Moves the turtle to the goal."""
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        print("Heading to x: %f, y: %f" % (goal_pose.x, goal_pose.y))
        distance_tolerance = 0.001

        vel_msg = Twist()
        while clamp(self.steering_angle(goal_pose) - self.pose.theta) >= distance_tolerance:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        while self.euclidean_distance(goal_pose) > distance_tolerance:

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        self.velocity_publisher.publish(vel_msg)
        return

        # If we press control + C, the node will stop.
        # rospy.spin()

    def drive_circle(self):
        vel_msg = Twist()
        start = time.time()
        while time.time()-start <= 2.1*math.pi:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.radius
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 1

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        return 


    def drive_square(self):
        goal1 = Pose()
        goal2 = Pose()
        goal3 = Pose()
        goal1.x = self.original_pose.x + self.side_length
        goal1.y = self.original_pose.y
        goal2.x = self.original_pose.x + self.side_length
        goal2.y = self.original_pose.y + self.side_length
        goal3.x = self.original_pose.x 
        goal3.y = self.original_pose.y + self.side_length
        
        self.move2goal(goal1)
        self.move2goal(goal2)
        self.move2goal(goal3)
        self.move2goal(self.original_pose)


    def drive_waypoints(self):
        # TODO
        pass


def clamp(angle):
    """
    Clamp angles between [-pi, p1]
    """
    while angle > math.pi:
        angle -= 2*math.pi
    
    while angle <= -math.pi:
        angle += 2*math.pi
    
    return angle


def drive_turtle_station(msg):
    """
    msg:
        string task
        float64 radius
        float64 length
        float64[] waypoints
        ---
        bool indicator
    """
    turtle = TurtleBot(msg.radius, msg.length, msg.waypoints)
    if msg.task == 'circle':
        print("turtle running in circle")
        turtle.drive_circle()
        return True
    elif msg.task == 'square':
        print("turtle running in square")
        turtle.drive_square()
        return True
    elif msg.task == 'custom':
        print("turtle following points")
        turtle.drive_waypoints()
        return True
    return False



if __name__ == '__main__':
    try:
        s = rospy.Service('turtle_drive', DriveTurtleSrv, drive_turtle_station)
        rospy.init_node('turtle_drive_server')
        print("Server is running")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass