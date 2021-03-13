#!/usr/bin/env python
# coding=utf-8


# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path


class Turtlebot():
    def __init__(self):
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.min_distance_from_obs = 0
        self.max_linear_speed = 0
        self.max_angular_speed = 0
        self.goal_tolerance = 0
        self.angle_to_limit = 0
        self.listener = tf.TransformListener()
        self.final_ranges = []
        self.goals = []
        self.laser_init = False
        self.laser = 0
        # Subscriber to receive info from LaserScan
        rospy.Subscriber("/scan", LaserScan, self.callback, queue_size=10)
        # Subscriber to receive info from PoseStamped
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.alert_point, queue_size=10)
        # Subscriber to receive info from Path
        rospy.Subscriber("/move_to_path", Path, self.alert_path, queue_size=10)

    def callback(self, data):
        self.laser = data
        self.laser_init = True
        # rospy.loginfo("Laser received " + str(data.ranges))

    def alert_point(self, data):
        new_point = {'x': data.pose.position.x, 'y': data.pose.position.y}
        self.goals.insert(0, new_point)
        print('Goal to reach: ', self.goals[0].get('x'), ', ', self.goals[0].get('y'))

    def alert_path(self, data):
        i = 0
        for p in data.poses:
            self.goals.insert(i, {'x': p.pose.position.x, 'y': p.pose.position.y})
            i += 1
        print('Goal to reach: ', self.goals[0].get('x'), ', ', self.goals[0].get('y'))

    def command(self, gx, gy):
        """
        Function that takes the robot to a point given by parameter
        :param gx: x-coordinate
        :param gy: y-coordinate
        """
        goal = PointStamped()
        base_goal = PointStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = rospy.Time()
        goal.point.x = gx
        goal.point.y = gy
        goal.point.z = 0.0
        try:
            base_goal = self.listener.transformPoint('base_footprint', goal)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Problem TF")
            return
        # TODO: put the control law here
        radianes = math.atan2(base_goal.point.y, base_goal.point.x)
        # grados = radianes * (180 / math.pi)
        distancia = math.sqrt(base_goal.point.x ** 2 + base_goal.point.y ** 2)
        angular = radianes
        # Check if the robot has reached the point
        if distancia < self.goal_tolerance:
            linear = 0
            angular = 0
            self.goals.pop(0)
            if len(self.goals) > 0:
                print('Goal to reach: ', self.goals[0].get('x'), ', ', self.goals[0].get('y'))
            else:
                print('Path finished, waiting for new points...')
        # We reduce speed proportionally to the remaining distance to reach the point
        elif distancia < self.max_linear_speed:
            linear = distancia
        # Robot goes forward
        else:
            linear = self.max_linear_speed
        if self.laser_init:
            self.limit_ranges()
            # If there is a obstacle near, the robot turns to the same direction (Bug algorithm)
            if min(self.final_ranges) < self.min_distance_from_obs:
                angular = self.max_angular_speed
                linear = 0
        """print("Distancia restante: ", distancia)
        print("Velocidad: ", linear)
        print("Radianes: ", radianes)
        print("Grados: ", grados)"""
        self.publish(linear, angular)

    def publish(self, lin_vel, ang_vel):
        """
        Set the linear speed and angular speed
        :param lin_vel: linear speed
        :param ang_vel: angular speed
        """
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = lin_vel
        # let's turn at 0 radians/s
        move_cmd.angular.z = ang_vel

        self.cmd_vel.publish(move_cmd)

    def limit_ranges(self):
        """
        Function that limits the viewing range of the laser to the degrees entered by parameter.
        :param ranges: 360° laser reading
        :param degrees: degrees to be limited
        """
        self.final_ranges = []
        for x in self.laser.ranges[:self.angle_to_limit + 1]:
            self.final_ranges.append(x)
        for x in self.laser.ranges[360 - self.angle_to_limit:]:
            self.final_ranges.append(x)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('robotcontrol', anonymous=False)
        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        robot = Turtlebot()
        # What function to call when you ctrl + c
        rospy.on_shutdown(robot.shutdown)
        # TODO 2: extend it to load more than one goal with base in "path" parameter
        if rospy.has_param('~path'):
            dic = rospy.get_param('~path/')
            for key in sorted(dic):
                robot.goals.append(dic.get(key))
            print('Goal to reach: ', robot.goals[0].get('x'), ', ', robot.goals[0].get('y'))
        # TODO 1: Load more internal parameters such as maximum speed, goal tolerance....
        robot.max_linear_speed = 0.18
        robot.max_angular_speed = 0.4
        robot.min_distance_from_obs = 0.25
        robot.goal_tolerance = 0.01
        robot.angle_to_limit = 55
        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)
        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            # rospy.loginfo("Loop")
            # publish the velocity
            if len(robot.goals) != 0:
                goalx = robot.goals[0].get('x')
                goaly = robot.goals[0].get('y')
                robot.command(goalx, goaly)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
    except:
        rospy.loginfo("robotcontrol node terminated ")
