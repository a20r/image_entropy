#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, Twist

ps = PoseStamped()
counter = 0
beta = 0
pose_pub = None
frame_id = None
altitude = None


def twist_callback(twist):
    global ps, pose_pub, counter, frame_id, beta
    ps.header.seq = counter
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = frame_id
    ps.pose.position.x += twist.linear.x
    ps.pose.position.y += twist.linear.y
    ps.pose.position.z = altitude
    beta += math.radians(4 * twist.angular.z)
    q = tf.transformations.quaternion_from_euler(0, 0, beta)
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
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
