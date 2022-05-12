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
        self.square_corners = [self.original_pose]*4
        print("hererererer")
        print(self.square_corners)


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

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self, goal_pose):
        """Moves the turtle to the goal."""
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.01

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
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
        self.square_corners[0].x += self.side_length
        self.square_corners[1].x += self.side_length
        self.square_corners[1].y += self.side_length
        self.square_corners[2].y += self.side_length
        print(self.square_corners[0])
        for i in range(4):
            self.move2goal(self.square_corners[i])


    def drive_waypoints(self):
        # TODO
        pass


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