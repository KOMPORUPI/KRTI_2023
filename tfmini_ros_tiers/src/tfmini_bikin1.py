#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)
distance = 0

def publisher_node():
    rospy.init_node('my_publisher1', anonymous=True)
    pub = rospy.Publisher('my_topic1', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 1 Hz

    while not rospy.is_shutdown():

        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)
            ser.reset_input_buffer()
            if recv[0] == 'Y' and recv[1] == 'Y': # 0x59 is 'Y'
                low = int(recv[2].encode('hex'), 16)
                high = int(recv[3].encode('hex'), 16)
                distance = low + high * 256

        # rospy.loginfo(distance)
        pub.publish(distance)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass