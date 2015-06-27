#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist

ps = PoseStamped()
counter = 0
pose_pub = None
frame_id = None
altitude = None


def twist_callback(twist):
    global ps, pose_pub, counter, frame_id
    ps.header.seq = counter
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = frame_id
    ps.pose.position.x += twist.linear.x
    ps.pose.position.y += twist.linear.y
    ps.pose.position.z = altitude
    ps.pose.orientation.x += twist.angular.x
    ps.pose.orientation.y += twist.angular.y
    ps.pose.orientation.z += twist.angular.z
    pose_pub.publish(ps)
    counter += 1


def run():
    global pose_pub, frame_id, altitude
    rospy.init_node('twist_to_pose', anonymous=False)
    twist_topic = rospy.get_param("~twist_topic")
    pose_topic = rospy.get_param("~pose_topic")
    frame_id = rospy.get_param("~frame_id")
    altitude = rospy.get_param("~altitude")
    rospy.Subscriber(twist_topic, Twist, twist_callback)
    pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=0)
    rospy.spin()

if __name__ == '__main__':
    run()
