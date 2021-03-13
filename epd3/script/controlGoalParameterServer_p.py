#!/usr/bin/env python


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


class Turtlebot():
    def __init__(self):

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.min_distance_from_obs = 0.22
        self.laser_init = False
        rospy.Subscriber("/scan", LaserScan, self.callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.alert_point)
        self.listener = tf.TransformListener()
        self.final_ranges = []
        self.path_completed = False
        self.goal_index = 0
        self.dict_goals = {}
        self.list_goals = []

    def get_dict_goals(self):
        return self.dict_goals

    def set_dict_goals(self, value):
        self.dict_goals = value
        self.list_goals = sorted(self.dict_goals)

    def add_goal(self, value):
        self.list_goals.append(value)

    def get_list_goals(self):
        return self.list_goals

    def get_current_goal_x(self):
        return self.dict_goals.get(self.get_list_goals()[self.goal_index]).get('x')

    def get_current_goal_y(self):
        return self.dict_goals.get(self.get_list_goals()[self.goal_index]).get('y')

    def callback(self, data):
        self.laser = data
        self.laser_init = True
        """i = 0
        for d in self.laser.ranges:
            if d != min(self.laser.ranges):
                i = i + 1
            else:
                print("posicion:", i, ", valor:", d)"""
        # rospy.loginfo("Laser received " + str(data.ranges))

    def alert_point(self, data):
        dic = {}
        dic['x'] = data.pose.position.x
        dic['y'] = data.pose.position.y
        self.get_dict_goals()["new_goal"] = dic
        print(self.get_dict_goals())

    def command(self, gx, gy):
        # rospy.loginfo("Command")
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
        grados = radianes * (180 / math.pi)
        distancia = math.sqrt(base_goal.point.x ** 2 + base_goal.point.y ** 2)
        angular = radianes / 1.2
        if distancia < 0.01:
            linear = 0
            angular = 0
            self.goal_index += 1
        elif distancia < 0.18:
            linear = distancia
        else:
            linear = 0.18
        if self.laser_init:
            self.split_ranges(self.laser.ranges, 45)
            if min(self.final_ranges) < self.min_distance_from_obs:
                angular = 0.4
                linear = 0
        """print("Distancia restante: ", distancia)
        print("Velocidad: ", linear)
        print("Radianes: ", radianes)
        print("Grados: ", grados)"""
        self.publish(linear, angular)

    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = lin_vel
        # let's turn at 0 radians/s
        move_cmd.angular.z = ang_vel

        self.cmd_vel.publish(move_cmd)

    def split_ranges(self, ranges, degrees):
        self.final_ranges = []
        for x in ranges[:degrees + 1]:
            self.final_ranges.append(x)
        for x in ranges[360 - degrees:]:
            self.final_ranges.append(x)
        #print("MIN: ", min(self.final_ranges))
        # print(self.final_ranges)

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

        # goalx = 2  # float(sys.argv[1])
        # goaly = 2  # float(sys.argv[2])

        # TODO 2: extend it to load more than one goal with base in "path" parameter
        if rospy.has_param('~path'):
            dic = rospy.get_param('~path/')
            robot.set_dict_goals(dic)
            print(robot.get_dict_goals())

        # TODO 1: Load more internal parameters such as maximum speed, goal tolerance....

        # print(' Goal to reach: ', goalx, ', ', goaly)

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            # rospy.loginfo("Loop")
            # publish the velocity
            # print(dic.get(sorted(dic)[robot.goal_index]))
            goalx = robot.get_current_goal_x()
            goaly = robot.get_current_goal_y()
            print(' Goal to reach: ', goalx, ', ', goaly)
            robot.command(goalx, goaly)
            #print(len(robot.get_dict_goals()))
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    except:
        rospy.loginfo("robotcontrol node terminated.")
