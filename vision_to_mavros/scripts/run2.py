#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.srv import CommandBool, SetMode

class DroneController:
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()

        self.target_distance = 1.0  # Desired distance to move in meters
        self.move_started = False

        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    def state_callback(self, state):
        self.current_state = state

    def pose_callback(self, pose):
        self.current_pose = pose

    def takeoff(self):
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')

        self.set_mode_service(custom_mode='GUIDED')
        self.arm_service(True)

    def move_forward(self):
        if not self.move_started:
            self.move_started = True
            self.target_pose = PoseStamped()
            self.target_pose.header.stamp = rospy.Time.now()
            self.target_pose.header.frame_id = 'base_link'  # Change to appropriate frame
            self.target_pose.pose.position.x = self.current_pose.pose.position.x + self.target_distance
            self.target_pose.pose.position.y = self.current_pose.pose.position.y
            self.target_pose.pose.position.z = self.current_pose.pose.position.z

        self.setpoint_pub.publish(self.target_pose)

    def land(self):
        self.set_mode_service(custom_mode='AUTO.LAND')

    def control_loop(self):
        rate = rospy.Rate(10)  # 10 Hz
        self.takeoff()

        while not rospy.is_shutdown():
            if self.current_state.mode == 'GUIDED' and self.current_state.armed:
                if not self.move_started:
                    self.move_forward()
                else:
                    distance_moved = abs(self.target_pose.pose.position.x - self.current_pose.pose.position.x)

                    if distance_moved >= self.target_distance:
                        rospy.loginfo("Reached the target distance.")
                        self.land()
                        break

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('drone_control_node')
    controller = DroneController()
    controller.control_loop()
