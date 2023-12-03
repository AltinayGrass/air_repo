#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import math
import tf
from tf.transformations import euler_from_quaternion
import sys,getopt

distance = 0.0
theta = 0.0
ctrl = 0.0

class PID:
    def __init__(self, Kp, Ki, Kd, Imax=sys.float_info.max, Imin=sys.float_info.min):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Imax= Imax
        self.Imin= Imin
        self.last_error = 0
        self.integral = 0

    def update(self, error, dt):
        derivative = (error - self.last_error) / dt
        self.integral += error * dt
        if self.integral > self.Imax:
            self.integral=self.Imax
        if self.integral < self.Imin:
            self.integral=self.Imin
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output


def move_robot(distance, max_speed, acceleration):
    global ctrl 
    rospy.init_node('my_robot_controller', anonymous=True)
    sub = tf.TransformListener()
    pub = rospy.Publisher('/diffbot/mobile_base_controller/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    hz=100
    rate = rospy.Rate(hz)  # 10 Hz
    
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    
    
    acceleration_time = max_speed / acceleration
    deceleration_time_distance = max_speed * acceleration_time / 2.0
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
    ref_distance = 0.0
    current_speed = 0.0
    ref_speed = 0.0
    pid_th = PID (Kp = 2.0, Ki = 0.02 , Kd = 0.5)
    pid_pos = PID (Kp = 2.0, Ki = 0.3 , Kd = 0.2 , Imax=0.5, Imin=-0.5)
    
    init=0

    while (time < acc_and_constant_speed_time + acceleration_time):
            try:
                (trans, rot) = sub.lookupTransform('/base_footprint','/odom',rospy.Time(0))
            except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
                rate.sleep()
                continue
            if init==0:
                init=1
                sx=trans[0]
                sy=trans[1]
                (roll, pitch, yaw_setpoint) = euler_from_quaternion (rot)
            (roll, pitch, yaw) =  euler_from_quaternion(rot)
            error_th=yaw-yaw_setpoint
            control_th = pid_th.update(error_th,(1.0 / hz))
            cmd.angular.z += control_th * (1.0 / hz)
            dist=math.sqrt((trans[0]-sx)*(trans[0]-sx)+(trans[1]-sy)*(trans[1]-sy))
            error_pos=ref_distance-dist

            control_pos = pid_pos.update(error_pos,(1.0 / hz))
            if time<acc_and_constant_speed_time:
                ref_speed += acceleration * (1.0 / hz)
                ref_speed = min(target_speed,ref_speed)               
            else:
                ref_speed -= acceleration * (1.0 / hz)
            
            ref_distance += ref_speed * (1.0 / hz)

            current_speed = ref_speed + (control_pos * ctrl )                
            
            cmd.linear.x = dir * current_speed                 
                            
            #print(f"Time: {time:.2f}s, Distance: {current_distance:.2f}m, Speed: {current_speed:.2f}m/s")
            print(f"TF Dist: {dist:.4f}, Cur_Dis: {ref_distance:.4f},Err: {error_pos:.4f},Ctrl_Pos: {control_pos:.4f},Cur_Spd: {current_speed:.4f}")
            pub.publish(cmd)
            time += (1.0 / hz)
            rate.sleep()
    cmd.linear.x = 0.0
    #print("Robot stopped.")

def turn_robot(distance, max_speed, acceleration):
    global ctrl 
    rospy.init_node('my_robot_controller', anonymous=True)
    sub = tf.TransformListener()
    pub = rospy.Publisher('/diffbot/mobile_base_controller/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    hz=100
    rate = rospy.Rate(hz)  # 10 Hz
    
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    
    
    acceleration_time = max_speed / acceleration
    deceleration_time_distance = max_speed * acceleration_time / 2.0
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
    ref_distance = 0.0
    current_speed = 0.0
    ref_speed = 0.0
    pid_pos = PID (Kp = 2.0, Ki = 0.3 , Kd = 0.2 , Imax=0.5, Imin=-0.5)
    
    init=0

    while (time < acc_and_constant_speed_time + acceleration_time):
            try:
                (trans, rot) = sub.lookupTransform('/base_footprint','/odom',rospy.Time(0))
            except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
                rate.sleep()
                continue
            if init==0:
                init=1
                (roll, pitch, yaw_start) = euler_from_quaternion (rot)
            (roll, pitch, yaw) =  euler_from_quaternion(rot)
            dist=yaw-yaw_start
            error_pos=ref_distance-dist
            control_pos = pid_pos.update(error_pos,(1.0 / hz))

            if time<acc_and_constant_speed_time:
                ref_speed += acceleration * (1.0 / hz)
                ref_speed = min(target_speed,ref_speed)               
            else:
                ref_speed -= acceleration * (1.0 / hz)
            
            ref_distance += ref_speed * (1.0 / hz)

            current_speed = ref_speed + (control_pos * ctrl )                
            
            cmd.angular.z = dir * current_speed                 
                            
            #print(f"Time: {time:.2f}s, Distance: {current_distance:.2f}m, Speed: {current_speed:.2f}m/s")
            print(f"TF Dist: {dist:.4f}, Cur_Dis: {ref_distance:.4f},Err: {error_pos:.4f},Ctrl_Pos: {control_pos:.4f},Cur_Spd: {current_speed:.4f}")
            pub.publish(cmd)
            time += (1.0 / hz)
            rate.sleep()
    cmd.angular.z = 0.0
    #print("Robot stopped.")



def main(argv):
    global distance
    global theta
    global ctrl
    try:
        opts, args = getopt.getopt(argv,"hd:t:c:",["distance=","theta=","control="])
    except getopt.GetoptError:
        print ('-d <in m> -c <on/off>')
        sys.exit(2)
    for opt,arg in opts:
        if opt == '-h':
            print ('-d <in m> -c <on/off>')
            sys.exit()
        elif opt in ("-d","--distance"):
            distance = float(arg)
        elif opt in ("-t","--theta"):
            theta = float(arg)
        elif opt in ("-c", "--control"):
            if arg=='on':
                ctrl = 1.0
            else:
                ctrl = 0.0
        
if __name__ == '__main__':
    try:
        main(sys.argv[1:])
        max_speed = 0.3  # Maksimum hız (pozitif)
        acceleration = 0.6  # Hızlanma (eşit ivme)
        max_th_speed = 4.0
        acc_th = 3.0
        move_robot(distance, max_speed, acceleration)
        turn_robot(theta,max_th_speed,acc_th)
    except KeyboardInterrupt:
        pass