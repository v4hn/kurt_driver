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
from geometry_msgs.msg import PoseStamped, Quaternion
import tf

broadcaster = None

def callback(msg_in):
    (r, p, y) = tf.transformations.euler_from_quaternion(
        [msg_in.orientation.x,
         msg_in.orientation.y,
         msg_in.orientation.z,
         msg_in.orientation.w])

    # mirror orientations
    q = tf.transformations.quaternion_from_euler(-r, -p, -y)    
    rotation = (q[0], q[1], q[2], q[3])

    broadcaster.sendTransform((0, 0, 0), rotation, msg_in.header.stamp, "/imu_frame", msg_in.header.frame_id)
    #broadcaster.sendTransform((0, 0, 0), rotation, rospy.Time.now(), "/imu_frame", msg_in.header.frame_id)

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
