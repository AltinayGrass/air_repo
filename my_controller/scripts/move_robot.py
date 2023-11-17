#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math
import tf
from tf.transformations import euler_from_quaternion
import sys

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def update(self, error, dt):
        derivative = (error - self.last_error) / dt
        self.integral += error * dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output


def move_robot(distance, max_speed, acceleration):

    rospy.init_node('my_robot_controller', anonymous=True)
    sub = tf.TransformListener()
    pub = rospy.Publisher('/diffbot/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    hz=10
    rate = rospy.Rate(hz)  # 10 Hz
    
    cmd = Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    
    
    acceleration_time = abs(max_speed) / acceleration
    deceleration_time_distance = abs(max_speed) * acceleration_time / 2.0
    constant_speed_distance = abs(distance) - 2.0 * deceleration_time_distance

    if constant_speed_distance < 0 :
        target_speed = math.sqrt(abs(distance) * acceleration)
        acceleration_time = target_speed / acceleration
        deceleration_time_distance = target_speed * acceleration_time / 2.0
        acc_and_constant_speed_time = acceleration_time
    else:
        target_speed = max_speed
        acceleration_time = target_speed / acceleration
        deceleration_time_distance = target_speed * acceleration / 2.0
        acc_and_constant_speed_time = acceleration_time + (constant_speed_distance / target_speed)
    
    #print(f"acc_and_constant_speed_time: {acc_and_constant_speed_time:.2f}s, Dec_Distance: {deceleration_time_distance:.2f}m, Target Speed: {target_speed:.2f}m/s")
    #input("Press Enter to continue...")

    dir = 1.0 
    if distance < 0:
        dir= -1.0

    time = 0.0
    current_distance = 0.0
    current_speed = 0.0
    pid_th = PID (Kp=-2.0, Ki=-0.02 , Kd=-0.5)
    yaw_setpoint=0.0

    while (time < acc_and_constant_speed_time + acceleration_time):
            if time<acc_and_constant_speed_time:
                current_speed += dir * acceleration * (1.0 / hz) 
                cmd.linear.x = dir * min(abs(target_speed),abs(current_speed))                 
            else:
                current_speed -= dir * acceleration * (1.0 / hz)
                cmd.linear.x = dir * max(0.0, abs(current_speed))                 
            pub.publish(cmd)
            rate.sleep()
            time += (1.0 / hz)
            current_distance += dir * cmd.linear.x * (1.0 / hz)
            current_speed = cmd.linear.x
            #print(f"Time: {time:.2f}s, Distance: {current_distance:.2f}m, Speed: {current_speed:.2f}m/s")
            try:
                (trans,rot) = sub.lookupTransform('/base_footprint','/odom',rospy.Time(0))
            except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
                continue
            (roll, pitch, yaw) = euler_from_quaternion(rot)
            error=yaw_setpoint-yaw
            control_signal = pid_th.update(error,(1.0 / hz))
            cmd.angular.z += control_signal * (1.0 / hz)
            print( yaw, error, cmd.angular.z)
    cmd.linear.x = 0.0
    #print("Robot stopped.")

if __name__ == '__main__':
    try:
        if sys.argv[1]:
            distance = float(sys.argv[1])
        else:
            distance = -0.5  # Hedef mesafe (0.8 metre, negatif)
        max_speed = 0.3  # Maksimum hız (pozitif)
        acceleration = 0.6  # Hızlanma (eşit ivme)
        move_robot(distance, max_speed, acceleration)
    except KeyboardInterrupt:
        pass