#!/usr/bin/env python


import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from std_msgs.msg import String

key = ""


def key_callback(data):
    global key
    key = data.data  
    rospy.loginfo(key)



def main():
    rospy.init_node('play_node')
    rc_override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=100)
    rospy.Subscriber('/keypress', String, key_callback)

    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('/mavros/cmd/arming')

    rc_override_msg = OverrideRCIn()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        if key == "a":
            # Arm the vehicle
            arming_client(True)
            rospy.loginfo("Vehicle armed.")
            global key
            key = "="
        elif key == "z":
            rc_override_msg.channels[8] = 1400
        elif key == "s":
            rc_override_msg.channels[8] = 1100
        elif key == "d":
            rc_override_msg.channels[8] = 1055
            arming_client(False)
            rospy.loginfo("Vehicle disarmed.")
            global key
            key = "="

        rc_override_pub.publish(rc_override_msg)
        # rospy.loginfo(key)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
