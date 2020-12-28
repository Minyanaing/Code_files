#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import math

from threading import Lock
import threading

from time import sleep

from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
import oriental_driver_Code as oriental
from nav_msgs.msg import Odometry
import tf

human_mutex = Lock()
fusion_mutex = Lock()

d_file = open ('/home/ubuntu/Desktop/rpm.txt', 'w')
d_file.write("")
d_file.close()

f=open ('/home/ubuntu/Desktop/rpm_raw.txt', 'w')
f.write("")
f.close()

class main():
    def __init__(self):
        rospy.init_node("Robot_controller", anonymous=False)

        self.variable_init()
        self.oriental_init()

        rospy.Subscriber("/cmd_vel", Twist, self.command_velocity_callback)
        self.pub_robot_speed=rospy.Publisher("robot_cmd_vel", Twist, queue_size=50)
        self.rpm_vel_publisher=rospy.Publisher("/rpm_vel", Twist, queue_size=50)
        self.robot_odom=rospy.Publisher('/robot_odometry', Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.Timer(rospy.Duration(0.1), self.pub_callback)
        rospy.Timer(rospy.Duration(0.1), self.main_callback)
        rospy.spin()
        # while not rospy.is_shutdown():
        #     rospy.sleep(0.02)
        #     print("-----------------")
        
        rospy.on_shutdown(self.shutdown)

        
    def shutdown(self):
        self.d_file.close()
        self.raw.close()
        print("Stop")
        self.oriental.motor_stop_decel(self.right_motor)
        self.oriental.motor_stop_decel(self.left_motor)

    def variable_init(self):
        self.wheel_distance=0.38
        self.wheel_radius=0.0762
        self.right_motor=1
        self.left_motor=2

        self.d_file=open('/home/ubuntu/Desktop/rpm.txt', 'a+')
        self.raw=open('/home/ubuntu/Desktop/rpm_raw.txt', 'a+')
        self.data_save_time=rospy.Time.now()

        self.vx=0
        self.vw=0
        self.robot_speed=Twist()
        self.rpm_vel=Twist()
        self.left_wheel_speed=0
        self.right_wheel_speed=0
        self.left_wheel_rev_set=0
        self.right_wheel_rev_set=0
        self.left_wheel_rev=0
        self.right_wheel_rev=0
        self.thred_flag=0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.robot_x=0
        self.robot_y=0
        self.robot_th=0

        self.robot_vx=0
        self.robot_vy=0
        self.robot_vr=0

        self.right_wheel_rev=0
        self.left_wheel_rev=0
        
    def oriental_init(self):
        port = rospy.get_param("motorDriver_port", "/dev/oriental_motor")
        baudrate = rospy.get_param("motorDriver_baudrate", 115200)
        self.oriental = oriental.oriental_driver(baudrate, port, 1)

    def command_velocity_callback(self, data):
        human_mutex.acquire()
        self.vx=data.linear.x
        self.vw=data.angular.z
        human_mutex.release()

    # Kinematics
    def vel_to_speed(self, linear, angular):
        self.left_wheel_speed=float((linear - angular*self.wheel_distance/2)/self.wheel_radius)
        self.right_wheel_speed=float((linear + angular*self.wheel_distance/2)/self.wheel_radius)    
    def speed_to_vel(self, left, right):
        x=float(self.wheel_radius*(right+left)/2)
        w=float(self.wheel_radius*(right-left)/self.wheel_distance)
        return x, w

    # Convertions
    def rads_to_revm(self, rad):
        rev_per_minute=30*30*rad/np.pi
        rev_per_minute=int(rev_per_minute)
        if rev_per_minute>3000 or rev_per_minute<-3000:
            rev_per_minute=int(3000)
        elif abs(rev_per_minute)<100:
            rev_per_minute=0
        return abs(rev_per_minute)
    def revm_to_rads(self, rev):
        raidan_per_second=rev*np.pi/(30*30)
        raidan_per_second=float(raidan_per_second)
        return raidan_per_second

# -----------------------------------------------------------------
    def pub_callback(self, event):
        fusion_mutex.acquire()
        
        robot_speed=Twist()
        rpm_vel=Twist()

        left_wheel_rev=self.oriental.speed_rpm_reader(self.left_motor)
        right_wheel_rev=self.oriental.speed_rpm_reader(self.right_motor)

        if right_wheel_rev!=None and left_wheel_rev!=None:
            rpm_vel.linear.x=-left_wheel_rev
            rpm_vel.linear.y=right_wheel_rev
        
            robot_speed.linear.x, robot_speed.angular.z = self.speed_to_vel(self.revm_to_rads(rpm_vel.linear.x), self.revm_to_rads(rpm_vel.linear.y))
            
            rpm_vel.angular.x=robot_speed.linear.x
            rpm_vel.angular.y=robot_speed.angular.z
        self.rpm_vel_publisher.publish(rpm_vel)
        self.pub_robot_speed.publish(robot_speed)
        self.d_file.write("Time:"+str((rospy.Time.now()-self.data_save_time).to_sec())+ "    RPM-->("+str(rpm_vel.linear.x)+", "+str(rpm_vel.linear.y)+")")
        self.d_file.write("   Velocity_IN_OUT--->("+str(robot_speed.linear.x)+", "+str(robot_speed.angular.z)+") \n")
        print(rpm_vel.linear.x, " RPM_Read ", rpm_vel.linear.y)

        self.current_time=rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()

        delta_x = (robot_speed.linear.x*np.cos(self.robot_th)) * dt
        delta_y = (robot_speed.linear.x*np.sin(self.robot_th)) * dt
        delta_th = robot_speed.angular.z * dt

        self.robot_x += delta_x
        self.robot_y += delta_y
        self.robot_th += delta_th

        odom_quat = tf.transformations.quaternion_from_euler(0, 0 , self.robot_th)
       # odom_quat = [0, 0, 0, 0]

        self.odom_broadcaster.sendTransform(
            (self.robot_x, self.robot_y, 0.0),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp=self.current_time
        odom.header.frame_id="odom"
        odom.pose.pose=Pose(Point(self.robot_x, self.robot_y, 0), Quaternion(*odom_quat))
        odom.child_frame_id="base_link"
        odom.twist.twist=Twist(Vector3(robot_speed.linear.x, 0, 0), Vector3(0, 0, robot_speed.angular.z))
        self.robot_odom.publish(odom)
        self.last_time=self.current_time

        fusion_mutex.release()
# ------------------------------------------------------------------
    def main_callback(self, event):
        fusion_mutex.acquire()
        
        self.vel_to_speed(self.vx, self.vw)
        
        self.left_wheel_rev_set=self.rads_to_revm(self.left_wheel_speed)
        self.right_wheel_rev_set=self.rads_to_revm(self.right_wheel_speed)

        if self.right_wheel_speed>0 and self.left_wheel_speed>0:
            self.oriental.speed_rpm_setter_backward(self.left_motor, self.left_wheel_rev_set)
            self.oriental.speed_rpm_setter_forward(self.right_motor, self.right_wheel_rev_set)
            # print("--Forward--")
            
        elif self.right_wheel_speed<0 and self.left_wheel_speed<0:
            self.oriental.speed_rpm_setter_forward(self.left_motor, self.left_wheel_rev_set)
            self.oriental.speed_rpm_setter_backward(self.right_motor, self.right_wheel_rev_set)
            # print("--Backward--")
        
        elif self.right_wheel_speed>0 and self.left_wheel_speed<0:
            self.oriental.speed_rpm_setter_forward(self.right_motor, self.right_wheel_rev_set)
            self.oriental.speed_rpm_setter_forward(self.left_motor, self.left_wheel_rev_set)
            # print("--Rotate-CCW--")
        
        elif self.right_wheel_speed<0 and self.left_wheel_speed>0:
            self.oriental.speed_rpm_setter_backward(self.right_motor, self.right_wheel_rev_set)
            self.oriental.speed_rpm_setter_backward(self.left_motor, self.left_wheel_rev_set)
            # print("--Rotate-CW--")

        elif self.vx==0 and self.vw==0:
            # self.oriental.motor_stop_decel(self.right_motor)
            # self.oriental.motor_stop_decel(self.left_motor)
            self.oriental.motor_stop_instant(self.right_motor)
            self.oriental.motor_stop_instant(self.left_motor)
            # print("--STOP--")

        # print(-self.oriental.speed_rpm_reader(self.left_motor), "RPM", self.oriental.speed_rpm_reader(self.right_motor))
        # print(self.left_wheel_rev, "RPM", self.right_wheel_rev)
        # print("Speed -->", self.vx, self.vw, "Read_Speed",self.robot_speed.linear.x, self.robot_speed.angular.z)
        print("  ")
        print("  ")

        fusion_mutex.release()



if __name__ == "__main__":
    try: main()
    except rospy.ROSInterruptException: pass
