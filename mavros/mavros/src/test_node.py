#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool


def rosInfo():
    rospy.loginfo("masuk")

def arm_and_override_throttle():
    # Initialize the ROS node
    rospy.init_node('arm_and_override')
    rosInfo()
    # Create a publisher for sending the RC override commands
    rc_override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=100)

    # Create a service client for arming/disarming
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    # Wait for the services to become available
    rospy.wait_for_service('/mavros/cmd/arming')

    # Arm the vehicle
    arming_client(True)
    rospy.loginfo("Vehicle armed.")

    # Create an OverrideRCIn message
    rc_override_msg = OverrideRCIn()

    # Set the throttle channel to 1400 (us)
    rc_override_msg.channels[2] = 1400


    # Publish the RC override message for a few seconds
    rate = rospy.Rate(10)  # 10 Hz
    range = 0
    while range < 100:  # Publish for 2 seconds
        rc_override_pub.publish(rc_override_msg)
        rate.sleep()
        range += 1

    rospy.loginfo("motor done")


    # Disarm the vehicle
    arming_client(False)
    rospy.loginfo("Vehicle disarmed.")



if __name__ == '__main__':
    try:
        arm_and_override_throttle()
    except rospy.ROSInterruptException:
        pass
