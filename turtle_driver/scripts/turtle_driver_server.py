#!/usr/bin/env python3

from cgitb import reset
import turtle

from yaml import YAMLError
import rospy
import time
from PID import PID
from geometry_msgs.msg import Twist, PoseStamped 
from turtlesim.msg import Pose
from nav_msgs.msg import Path
import math
from turtle_driver.srv import DriveTurtleSrv
from std_srvs.srv import Empty
from rosgraph_msgs.msg import Log

class TurtleBot:

    def __init__(self, radius, side_length, waypoints):
        """
        Attributes:
            pose: turtle current pose received from topic
            rate: global topic publishing rate
            radius: radius of circular movement
            side_length: side length of square movement
            waypoints: waypoints following
            linear_PID: to control the linear velocity
        """

        turtle_cmd_vel = rospy.get_param('turtle_cmd_vel')
        turtle_pose = rospy.get_param('turtle_pose')

        self.velocity_publisher = rospy.Publisher(turtle_cmd_vel,
                                                  Twist, queue_size=10)
        self.pose_publisher = rospy.Publisher('turtle_current_pose', PoseStamped, queue_size=20)

        self.pose_subscriber = rospy.Subscriber(turtle_pose,
                                              Pose, self.update_pose)
        self.rosout_subscriber = rospy.Subscriber('/rosout',
                                              Log, self.need_reset)
        self.pose = Pose()
        self.rate = rospy.Rate(10)  # 10Hz
        self.radius = radius
        self.side_length = side_length
        self.waypoints = waypoints
        self.linear_PID = PID()
        self.need_reset = False

    def need_reset(self, data):
        if data.msg[:2] == 'Oh':
            self.need_reset = True

    def update_waypoints(self):
        """
        Returns:
            A list of tuple (x, y) as waypoints
        """
        waypoints = []
        for i in range(len(self.waypoints.poses)):
            x = self.waypoints.poses[i].pose.position.x
            y = self.waypoints.poses[i].pose.position.y
            waypoints.append((x, y))
        return waypoints

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.turtule_pose2PoseStamped()

    def turtule_pose2PoseStamped(self):
        PoseStamped_foo = PoseStamped()
        PoseStamped_foo.header.frame_id = "turtle_current_pose_frame"
        PoseStamped_foo.pose.position.x = self.pose.x
        PoseStamped_foo.pose.position.y = self.pose.y
        self.pose_publisher.publish(PoseStamped_foo)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        distance = math.sqrt(math.pow((goal_pose.x - self.pose.x), 2) +
                    math.pow((goal_pose.y - self.pose.y), 2))
        return distance

    def steering_angle(self, goal_pose):
        """Calculate the angle between goal and current theta"""
        steering_angle = math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
        return steering_angle

    def linear_vel(self, goal_pose):
        """Calculate linear velocity to the goal"""
        error = self.euclidean_distance(goal_pose)
        linear_vel = self.linear_PID.update_vel(error)
        return linear_vel


    def angular_vel(self, goal_pose, constant=6):
        "Calculate angular velocity to the goal"
        goal_theta = self.steering_angle(goal_pose)
        return constant * clamp(goal_theta - self.pose.theta)

    def move2goal(self, goal_pose):
        """Moves the turtle to a given goal."""

        vel_msg = Twist()

        while abs(clamp(self.steering_angle(goal_pose) - self.pose.theta)) >= 0.001:
            if self.need_reset:
                return False
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.velocity_publisher.publish(vel_msg)


        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        while self.euclidean_distance(goal_pose) >= 0.01:
            if self.need_reset:
                return False
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            self.velocity_publisher.publish(vel_msg)

        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)
        return True

    def drive_circle(self):
        """Moves the turtle in a circle with self.radius"""
        vel_msg = Twist()
        start = time.time()
        while time.time()-start <= 2.1*math.pi:
            if self.need_reset:
                return False
            vel_msg.linear.x = self.radius
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 1

            self.velocity_publisher.publish(vel_msg)

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        return True

    def drive_square(self):
        """Moves the turtle in a square with self.side_length"""
        goal= Pose()


        goal.x = self.pose.x + self.side_length
        goal.y = self.pose.y
        if not self.move2goal(goal): return False
        
        goal.x = self.pose.x
        goal.y = self.pose.y + self.side_length
        if not self.move2goal(goal): return False

        goal.x = self.pose.x - self.side_length
        goal.y = self.pose.y
        if not self.move2goal(goal): return False

        goal.x = self.pose.x
        goal.y = self.pose.y - self.side_length
        if not self.move2goal(goal): return False

        return True

    def drive_waypoints(self):
        """Moves the turtle follwing waypoints"""
        waypoints_lst = self.update_waypoints()
        for i in range(len(waypoints_lst)):
            goal = Pose()
            goal.x = waypoints_lst[i][0]
            goal.y = waypoints_lst[i][1]
            if self.move2goal(goal):
                continue
            else:
                return False
        return True


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
        nav_msgs/Path[] waypoints
        ---
        bool indicator
    """
    turtle = TurtleBot(msg.radius, msg.length, msg.waypoints)
    time.sleep(1)
    if msg.task == 'circle':
        print("turtle running in circle")
        if turtle.drive_circle():
            return True
        else:
            reset_turtle()
            return False
    elif msg.task == 'square':
        print("turtle running in square")
        if turtle.drive_square():
            return True
        else:
            reset_turtle()
            return False
    elif msg.task == 'custom':
        print("turtle following points")
        if turtle.drive_waypoints():
            return True
        else:
            reset_turtle()
            return False
    elif msg.task == 'reset':
        print("Reseting...")
        reset_turtle()
    return False


def reset_turtle():
    clear_bg = rospy.ServiceProxy('reset', Empty)
    clear_bg()


if __name__ == '__main__':
    try:
        rospy.init_node('turtle_drive_node')
        s = rospy.Service('turtle_drive', DriveTurtleSrv, drive_turtle_station)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass    