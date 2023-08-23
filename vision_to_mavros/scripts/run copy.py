#!/usr/bin/env python

##
#
# Control a MAV via mavros, t265 only for positioning
#
##

import rospy
import tf
import math
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion



pi_2 = 3.141592654 / 2.0
wp = 0

OVERRIDE_CHANNEL_LIST = [0,1,2,3,8] #ro, pit, tht, ya, rly


ROLL_OVERRIDE_IDLE = [1521, 1521]
PITCH_OVERRIDE_IDLE = [1495, 1495]
THROTTLE_OVERRIDE_IDLE = [1565, 1565]
YAW_OVERRIDE_IDLE = [1506, 1506]

RELAY_OVERRIDE = [991, 2015]


ROLL_OVERRIDE_SLOW = [1409, 1650]
PITCH_OVERRIDE_SLOW = [1650, 1420]
THROTTLE_OVERRIDE_SLOW = [1455, 1695]
YAW_OVERRIDE_SLOW = [1408, 1650]

ROLL_OVERRIDE_FAST = [1272,1755]
PITCH_OVERRIDE_FAST = [1747, 1260]
THROTTLE_OVERRIDE_FAST = [1331, 1759]
YAW_OVERRIDE_FAST = [1284, 1752]

ROLL_OVERRIDE = [ROLL_OVERRIDE_IDLE,ROLL_OVERRIDE_SLOW,ROLL_OVERRIDE_FAST]
PITCH_OVERRIDE = [PITCH_OVERRIDE_IDLE, PITCH_OVERRIDE_SLOW,PITCH_OVERRIDE_FAST]
THROTTLE_OVERRIDE = [THROTTLE_OVERRIDE_IDLE, THROTTLE_OVERRIDE_SLOW,THROTTLE_OVERRIDE_FAST]
YAW_OVERRIDE = [YAW_OVERRIDE_IDLE, YAW_OVERRIDE_SLOW,YAW_OVERRIDE_FAST]


# ROLL_OVERRIDE_FAST = 1521
# PITCH_OVERRIDE_FAST = 1495
# THROTTLE_OVERRIDE_FAST = 1565
# YAW_OVERRIDE_FAST = 1506
# RELAY_OVERRIDE_FAST = 991



class MavController:

    """
    A simple object to help interface with mavros
    """

    def __init__(self):

        rospy.init_node("mav_control_node")

        # rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/vision_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)

        rospy.Subscriber("/my_topic", Int32, self.tfmini_callback)
        rospy.Subscriber("/my_topic1", Int32, self.tfmini_callback1)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_data_callback)


        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=100)
        self.attitude_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude', AttitudeTarget, queue_size=10)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND


        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)


        self.rc = RCIn()
        self.rc_override_msg = OverrideRCIn()
        self.pose_now = Pose()
        self.pose_cmd = Pose()
        self.timestamp = rospy.Time()
        self.mission = 0
        self.offset = 100
        self.tf_dist = 0
        self.tf_dist_side = 0
        self.rl = 0
        self.pi = 0
        self.ya = 0



        self.siding = "diem" #1 roll kiri # roll kanan 


        

    def tfmini_callback(self, data):
        self.tf_dist = data.data

    def tfmini_callback(self, data):
        self.tf_dist_side = data.data

        # rospy.loginfo(data)

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def imu_data_callback(self, imu_msg):
        angular_velocity = imu_msg.angular_velocity
        linear_acceleration = imu_msg.linear_acceleration
        orientation = imu_msg.orientation

        # Convert quaternion to Euler angles
        self.rl, self.pi, self.ya = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # rospy.loginfo("Orientation (roll, pitch, yaw): (%f, %f, %f)", math.degrees(self.rl), math.degrees(self.pi), math.degrees(self.ya))


    def override(self, state, channel, val, speed, is_tht=0):
        """
        Override RC
        """

        if speed == 1:
            speeds = 1
            dir = 1
        elif speed == -1:
            speeds = 1
            dir = 0
        elif speed == 2:
            speeds = 2
            dir = 1
        elif speed == -2:
            speeds = 2
            dir = 0
        elif speed == 0:
            speeds = 0
            dir = 0
        

        if state == 1:
            for i in range(0,18):
                if i == channel:
                    # rospy.loginfo("ini channel: %f ", channel)
                    self.rc_override_msg.channels[channel] = val[speeds][dir]

                    #keeps tht in idle
                    if is_tht == 0: 
                        self.rc_override_msg.channels[2] = THROTTLE_OVERRIDE_IDLE

                elif i in OVERRIDE_CHANNEL_LIST and not (channel or 2):
                    if i == 0 :
                        ovr = ROLL_OVERRIDE_IDLE[0]
                    if i == 1 :
                        ovr = PITCH_OVERRIDE_IDLE[0]
                    # if i == 2 :
                    #     ovr = THROTTLE_OVERRIDE_IDLE[0]
                    if i == 3 :
                        ovr = YAW_OVERRIDE_IDLE[0]

                    self.rc_override_msg.channels[i] = ovr
                


        if state == 0:
            for i in range(0,18):
                if i == channel:
                    self.rc_override_msg.channels[i] = val[0][0]
                elif i in OVERRIDE_CHANNEL_LIST and not channel:
                    self.rc_override_msg.channels[i] = val[0][0]

    def pitch(self,vel):
        self.override(1, OVERRIDE_CHANNEL_LIST[1], PITCH_OVERRIDE, vel)

        if vel == 0:
            self.override(0, OVERRIDE_CHANNEL_LIST[1], PITCH_OVERRIDE, vel)


    def yaw(self, vel):
        self.override(1, OVERRIDE_CHANNEL_LIST[3], YAW_OVERRIDE, vel)

        if vel == 0:
            self.override(0, OVERRIDE_CHANNEL_LIST[3], YAW_OVERRIDE, vel)


    def roll(self, vel):
        self.override(1, OVERRIDE_CHANNEL_LIST[0], ROLL_OVERRIDE, vel)

        if vel == 0:
            self.override(0, OVERRIDE_CHANNEL_LIST[0], ROLL_OVERRIDE, vel)
        

    def throttle(self, vel):

        self.override(1, OVERRIDE_CHANNEL_LIST[2], THROTTLE_OVERRIDE, vel)
        
        if vel == 0:
            self.override(0, OVERRIDE_CHANNEL_LIST[2], THROTTLE_OVERRIDE, vel)





    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)
    
    def movex(self, pose):
        self.set_vel(2,0,0)
    def movey(self, pose):
        self.set_vel(0,2,0)
    def movez(self, pose):
        self.set_vel(0,0,2)

    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quat = tf.transformations.quaternion_from_euler(ro, pi, ya + pi_2)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.goto(pose)
        #print(quat)

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        # Set to stabilize mode for arming
        # mode_resp = self.mode_service(custom_mode="0")
        mode_resp = self.mode_service(custom_mode="0")

        # rospy.sleep(2)
        self.arm()
        # print("armed")
        rospy.sleep(3)
        # print("takeoff")

        # Set to guided mode
        mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        #return takeoff_resp

        return mode_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()


    #masih redundant, full t265 no gps, not used yet
    def waypoint1(self):
        self.pose_cmd.position.y = 1000.0
        rospy.loginfo("here calc range")
        # pos_cmd = 5000
        pos_now = 6370.0 - float(self.tf_dist)
        

        # if self.tf_dist > pos_cmd - self.offset or pos_now < self.pose_cmd.position.y + self.offset:
        if pos_now > self.pose_cmd.position.y - self.offset or pos_now < self.pose_cmd.position.y + self.offset:
            # self.set_vel(0,0.5,0)

            self.rc_override_msg.channels[1] = 1400
            # self.rc_override_msg.channels[1] = 1400
            # rospy.loginfo(self.pose_now.position.y)
        else:
            self.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE
            # self.mission += 1

        
    def waypoint2(self):
        self.pose_cmd.position.z = 2

        if self.pose_now.position.z > self.pose_cmd.position.z + self.offset or self.pose_now.position.z < self.pose_cmd.position.z - self.offset:
            # self.set_vel(0,0,-.5)
            self.throttle(-1)
            # self.rc_override_msg.channels[1] = 1400
        else:
            self.mission += 1

    def waypoint3(self):
        self.pose_cmd.position.z = 5

        if self.pose_now.position.z > self.pose_cmd.position.z + self.offset or self.pose_now.position.z < self.pose_cmd.position.z - self.offset:
            self.throttle(1)
        else:
            self.mission += 1
    
    def waypoint4(self):
        self.pose_cmd.position.y = 5

        if self.pose_now.position.y > self.pose_cmd.position.y + self.offset or self.pose_now.position.y < self.pose_cmd.position.y - self.offset:
            self.pitch(1)
        else:
            self.mission += 1


    def is_balance(self):
        
        bal = False

        if self.ya > 1.35 and self.ya < 1.45:
            bal = True
        else:
            bal = False

        return bal

    def is_balance2(self):
        
        bal = False

        if self.ya > 2.85 and self.ya < 3.00:
            bal = True
        else:
            bal = False

        return bal



    def is_center(self):
        cen = False


        if self.tf_dist_side < 115:
            self.siding = "kanan"
        elif self.tf_dist_side > 125:
            self.siding = "kiri"





    # not working

    def set_attitude(self, roll_angle_degrees, pitch_angle_degrees, yaw_angle_degrees):
        roll_angle_radians = math.radians(roll_angle_degrees)
        pitch_angle_radians = math.radians(pitch_angle_degrees)
        yaw_angle_radians = math.radians(yaw_angle_degrees)
        
        attitude_msg = AttitudeTarget()
        attitude_msg.header.stamp = rospy.Time.now()
        attitude_msg.orientation = Quaternion(*quaternion_from_euler(roll_angle_radians, pitch_angle_radians, yaw_angle_radians))
        attitude_msg.type_mask = 7  # Only control roll, pitch, and yaw
        
        self.attitude_pub.publish(attitude_msg)




def quaternion_from_euler(roll, pitch, yaw):

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr
    
    return qx, qy, qz, qw


def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """

    c = MavController()


    r = rospy.Rate(100) # 10hz
    c.takeoff(1)
    rospy.sleep(2)
    
    rospy.loginfo("takeoff")
    # mode_resp = c.mode_service(custom_mode="5")

    c.mission = 1

    state = False
    state2 = False
    state3 = False



    # relay
    counter = 0

    stateMuter = False
    stateSudahMuter = False
    stateMenjatuhkanBarang = False
    stateSudahMundur = False


    print("terbang tunggu 2 det")
    rospy.sleep(2)
    print("sudah terbang")
    
    mode_resp = c.mode_service(custom_mode="5")

    while not rospy.is_shutdown():
        
        if c.is_balance():

            # c.throttle(0)
            print(c.ya)

            # if c.rc.channels[4] == 1606:
                # c.rc_override_msg.channels[0] = ROLL_OVERRIDE_IDLE[0]
                # c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
                # c.rc_override_msg.channels[2] = THROTTLE_OVERRIDE_IDLE[0]
                # c.rc_override_msg.channels[3] = YAW_OVERRIDE_IDLE[0]
                # break
            
            if c.tf_dist > 470 and c.tf_dist < 490 and state2 == False:

                state = True
                print("Berhenti...")
                state2 = True
            else:
                print("Nggak Masuk If")


            # if c.is_balance:

            #         # c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
            #     # else :

            #         # c.rc_override_msg.channels[1] = 1450
            #         # c.rc_override_msg.channels[2] = 1580        
            # else:
            #     print("Tidak Balance...")
            #     c.rc_override_msg.channels[0] = ROLL_OVERRIDE_IDLE[0]
            #     c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
            #     c.rc_override_msg.channels[2] = THROTTLE_OVERRIDE_IDLE[0]
            #     c.rc_override_msg.channels[3] = YAW_OVERRIDE_IDLE[0]
            

            if c.tf_dist < 200 and state2:
                print("Jarak di dibawah 200...")
                c.rc_override_msg.channels[0] = ROLL_OVERRIDE_IDLE[0]
                c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
                c.rc_override_msg.channels[2] = THROTTLE_OVERRIDE_IDLE[0]
                c.rc_override_msg.channels[3] = YAW_OVERRIDE_IDLE[0]
                state = True
                counter = 0
                break
                
            if state == True:
                print("True")
                c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
                if state3 == False:
                    c.rc_override_msg.channels[2] = 1413

                    c.rc_override_msg.channels[8] = RELAY_OVERRIDE[1]
                counter = counter + 1
            else:
                if c.is_balance():
                    c.rc_override_msg.channels[1] = 1350
                    c.rc_override_msg.channels[2] = 1610
                else:
                    c.rc_override_msg.channels[0] = ROLL_OVERRIDE_IDLE[0]
                    c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
                    c.rc_override_msg.channels[2] = THROTTLE_OVERRIDE_IDLE[0]
                    c.rc_override_msg.channels[3] = YAW_OVERRIDE_IDLE[0]

                print("False")

            print(c.tf_dist)
            
            if counter > 350:
                c.rc_override_msg.channels[8] = RELAY_OVERRIDE[1]
                c.rc_override_msg.channels[2] = 1650
                print("Counter > 500")
                
                state3 == True
                
            if counter > 800:
                print("MENYALAKAN MAGNET")
                c.rc_override_msg.channels[8] = RELAY_OVERRIDE[1]
                counter = 0
                state = False
            # rospy.loginfo(c.tf_dist)



        
        else:
            c.rc_override_msg.channels[0] = ROLL_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[2] = THROTTLE_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[3] = YAW_OVERRIDE_IDLE[0]
        
        c.rc_override.publish(c.rc_override_msg)
        r.sleep()
        
       

    counter2 = 0
    counter3 = 0
    counter4 = 0
    while not rospy.is_shutdown():
        print(c.ya)
        print(c.pi)
        print(c.tf_dist)
        if stateMuter == False:
            c.rc_override_msg.channels[0] = ROLL_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[2] = THROTTLE_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[3] = YAW_OVERRIDE_IDLE[0]
            print("MENYEIMBANGKAN")
            counter2 = counter2 + 1
        else:
            if not c.is_balance2() and stateSudahMuter == False:
                c.rc_override_msg.channels[3] = YAW_OVERRIDE_SLOW[0]
                print("MUTER")
            else:
                # print("STOP MUTER")
                stateSudahMuter = True

        
        if counter2 > 500:
            stateMuter = True


        if stateSudahMuter == True and c.tf_dist < 611:
            c.rc_override_msg.channels[1] = 1650
            c.rc_override_msg.channels[2] = 1580
            print("MUNDUR")
            # c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
            # c.rc_override_msg.channels[2] = THROTTLE_OVERRIDE_IDLE[0]

        if c.tf_dist >= 611:
            
            
            stateSudahMundur = True

        if stateSudahMundur == True:
            print("MENYEIMBANGKAN")
            c.rc_override_msg.channels[0] = ROLL_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[2] = THROTTLE_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[3] = YAW_OVERRIDE_IDLE[0]
            counter3 = counter3 + 1

        if counter3 > 500:
            stateMenjatuhkanBarang = True        

        if stateMenjatuhkanBarang == True:
            print("MENJATUHKAN BARANG LALU KE KANAN")
            c.rc_override_msg.channels[8] = RELAY_OVERRIDE[0]
            c.rc_override_msg.channels[0] = 1650
            counter4 = counter4 + 1

        if counter4 > 500:
            print("MENYEIMBANGKAN")
            c.rc_override_msg.channels[0] = ROLL_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[2] = THROTTLE_OVERRIDE_IDLE[0]
            c.rc_override_msg.channels[3] = YAW_OVERRIDE_IDLE[0]

        if counter4 > 1000:
            print("LANDING")
            break
            



        c.rc_override.publish(c.rc_override_msg)
        r.sleep()


    
    # c.land()
    # if
    # c.land()
    # if c.rc.channels[4] == 1606:
    #     c.land()
    # else:
    #     c.rc_override_msg.channels[0] = ROLL_OVERRIDE_IDLE[0]
    #     c.rc_override_msg.channels[1] = PITCH_OVERRIDE_IDLE[0]
    #     c.rc_override_msg.channels[2] = THROTTLE_OVERRIDE_IDLE[0]
    #     c.rc_override_msg.channels[3] = YAW_OVERRIDE_IDLE[0]
        
    #     c.rc_override.publish(c.rc_override_msg)

    
        


    # count = 0
    # c.relay(1)
    # while count < 1000:
    #      count += 1
    #     #  rospy.INFO(c.rc_override_msg)
    #      r.sleep()

    # c.relay(0)


    

    # print("Takeoff")
    # #first alt = 24
    # c.takeoff(5)


    # rospy.sleep(5)
    # c.waypoint1()
    # rospy.sleep(5)
    # c.waypoint2()
    # rospy.sleep(5)
    # c.waypoint3()
    # rospy.sleep(5)
    # c.waypoint4()
        


        # c.goto_xyz_rpy(0,0,1.2,0,0,0)
        # rospy.sleep(3)

        # print("Waypoint 1: position control")
        # c.goto_xyz_rpy(0.0,0.0,1.2,0,0,-1*pi_2)
        # rospy.sleep(2)
        # c.goto_xyz_rpy(0.4,0.0,1.2,0,0,-1*pi_2)
        # rospy.sleep(3)
        # print("Waypoint 2: position control")
        # c.goto_xyz_rpy(0.4,0.0,1.2,0,0,0)
        # rospy.sleep(2)
        # c.goto_xyz_rpy(0.4,0.4,1.2,0,0,0)
        # rospy.sleep(3)
        # print("Waypoint 3: position control")
        # c.goto_xyz_rpy(0.4,0.4,1.2,0,0,pi_2)
        # rospy.sleep(2)
        # c.goto_xyz_rpy(0.0,0.4,1.2,0,0,pi_2)
        # rospy.sleep(3)
        # print("Waypoint 4: position control")
        # c.goto_xyz_rpy(0.0,0.4,1.2,0,0,2*pi_2)
        # rospy.sleep(2)
        # c.goto_xyz_rpy(0.0,0.0,1.2,0,0,2*pi_2)
        # rospy.sleep(3)

        #print("Velocity Setpoint 1")
        # c.set_vel(0,0.1,0)
        # rospy.sleep(5)
        #print("Velocity Setpoint 2")
        # c.set_vel(0,-0.1,0)
        # rospy.sleep(5)
        #print("Velocity Setpoint 3")
        # c.set_vel(0,0,0)
        # rospy.sleep(5)

    # print("Landing")
    c.land()

if __name__=="__main__":
    simple_demo()
