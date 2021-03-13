#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path

if __name__ == '__main__':
    try:
        path_publisher = rospy.Publisher('/move_to_path', Path, queue_size=10)
        rospy.init_node('followingPath', anonymous=False)
        while not rospy.is_shutdown():
            path = Path()
            argv = raw_input('Write the new path following this syntax: \"x1,y1;x2,y2\":')
            for point in argv.split(';'):
                if len(point.split(",")) != 2:
                    print("Only 2 coordinates")
                    break
                else:
                    x, y = point.split(",")
                    pose = PoseStamped()
                    pose.pose.position.x = float(x)
                    pose.pose.position.y = float(y)
                    pose.pose.position.z = 0
                    path.poses.append(pose)
            path_publisher.publish(path)
            print("Appended new path")
    except:
        rospy.loginfo("Publisher node terminated")