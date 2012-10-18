#!/usr/bin/env python
'''
Created on 18.10.2012

@author: Martin Guenther <mguenthe@uos.de>

This ROS node publishes a TF frame for IMU messages (for visualization in
RViz). Note that ROS Fuerte+ has a RViz plugin that can directly visualize IMU
messages.
'''

import roslib
roslib.load_manifest('imu_recalibration')

import rospy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import tf

broadcaster = None

def callback(msg_in):
    msg_out = PoseStamped()

    msg_out.header = msg_in.header
    msg_out.pose.position.x = 0
    msg_out.pose.position.y = 0
    msg_out.pose.position.z = 0

    msg_out.pose.orientation = msg_in.orientation
    rotation = (msg_in.orientation.x, msg_in.orientation.y, msg_in.orientation.z, msg_in.orientation.w)

    #broadcaster.sendTransform((0, 0, 0), rotation, msg_in.header.stamp, msg_in.header.frame_id, "/imu_frame")
    broadcaster.sendTransform((0, 0, 0), rotation, rospy.Time.now(), msg_in.header.frame_id, "/imu_frame")

def main():
    global broadcaster
    rospy.init_node('imu_to_tf', anonymous=True)
    broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber('imu', Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
