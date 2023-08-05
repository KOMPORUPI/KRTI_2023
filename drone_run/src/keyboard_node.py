#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
import termios
import tty


def keypress_publisher():
    # Initialize the ROS node
    rospy.init_node('keypress_node')

    # Create a publisher
    pub = rospy.Publisher('keypress', String, queue_size=10)

    # Set the rate at which to publish the keypress events
    rate = rospy.Rate(10)  # 10 Hz

    # Get the file descriptor for standard input
    fd = sys.stdin.fileno()

    # Save the current terminal settings
    old_settings = termios.tcgetattr(fd)

    try:
        # Set the terminal settings to raw mode
        tty.setraw(fd)

        # Main loop
        while not rospy.is_shutdown():
            # Read a single character
            char = sys.stdin.read(1)
            # Publish the keypress event
            pub.publish(char)
            # Sleep to maintain the desired publishing rate
            rate.sleep()

    finally:
        # Restore the terminal settings
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    try:
        keypress_publisher()
    except rospy.ROSInterruptException:
        pass