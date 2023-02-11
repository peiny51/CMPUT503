#!/usr/bin/env python3

import os
import time

import rospy
import rosbag
from std_msgs.msg import String, Float32

from math import pi, cos, sin

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, WheelEncoderStamped
from duckietown_msgs.srv import GetVariable

# from exercise_two.srv import Led, LedResponse


class DuckieMover(DTROS):
    def __init__(self, node_name):
        super(DuckieMover, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        name = os.environ['VEHICLE_NAME']
        
        self.stop_pub = rospy.Publisher(f'/{name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size = 1)
        self.vel_pub = rospy.Publisher("/"+os.environ['VEHICLE_NAME']+"/joy_mapper_node/car_cmd", Twist2DStamped, queue_size = 1)

        self.right_tick_sub = rospy.Subscriber(f'/{name}/right_wheel_encoder_node/tick', 
        WheelEncoderStamped, self.right_tick,  queue_size = 1)
        self.left_tick_sub = rospy.Subscriber(f'/{name}/left_wheel_encoder_node/tick', 
        WheelEncoderStamped, self.left_tick,  queue_size = 1)

        self.bag = rosbag.Bag('/code/catkin_ws/src/503DuckieBot/packages/exercise_two/run.bag', 'w')

        self.r = rospy.get_param(f'/{name}/kinematics_node/radius', 100)
        rospy.loginfo("Radius of wheel: " +  str(self.r))

        self.Xw = 0.32
        self.Yw = 0.32
        self.Xr = 0
        self.Yr = 0
        self.Th = 0
        self.L = 0.05

        self.reset()

    def reset(self):
        self.rt_initial_set = False
        self.rt = 0
        self.rt_initial_val = 0

        self.lt_initial_set = False
        self.lt = 0
        self.lt_initial_val = 0


    def right_tick(self, msg):
        if not self.rt_initial_set:
            self.rt_initial_set = True
            self.rt_initial_val = msg.data
        self.rt = msg.data - self.rt_initial_val
        # self.rt_dist = (2 * pi * self.r * self.rt_val) / 135


    def left_tick(self, msg):
        if not self.lt_initial_set:
            self.lt_initial_set = True
            self.lt_initial_val = msg.data
        self.lt = msg.data - self.lt_initial_val
        # self.lt_dist = (2 * pi * self.r * self.lt_val) / 135

    def stop(self, duration):
        msg_velocity = Twist2DStamped()
        msg_velocity.header.stamp = rospy.Time.now()
        msg_velocity.v = 0
        msg_velocity.omega = 0
        self.vel_pub.publish(msg_velocity)
        # msg_txt = "Sending v: 0, omega = 0"
        # rospy.loginfo(msg_txt)
        time.sleep(duration)

    def ex_two_part_two(self):
        # State one: (Green led for 5s)
        rospy.loginfo("Waiting for led service..")
        rospy.wait_for_service('led_control_srv')
        rospy.loginfo("Connected to led service.")
        led_srv = rospy.ServiceProxy('led_control_srv', GetVariable)
        name_json = String()
        name_json.data = str("3")  # red
        led_srv(name_json)
        time.sleep(5)

        rad_90 = pi/2

        name_json.data = str("1")  # green
        led_srv(name_json)
        self.task_rotation(-rad_90, -(pi * 3.2), 0.25)       # 90 deg clockwise, 1st turn
        self.task_straight_125cm(0.105)                     # Go straight, bottom part 
        self.task_rotation(rad_90, pi * 1.6, 0.28)         # 90 deg counter clockwise, bottom right turn
        self.task_straight_125cm(0.09)              # Go straight, right part 
        self.task_rotation(rad_90, pi * 1.2, 0.2)   # 90 deg counter clockwise, middle right turn
        self.task_straight_125cm(0.085)              # Go straight, center-right to center-left part 

        name_json.data = str("3")  # red
        led_srv(name_json)
        time.sleep(5)

        name_json.data = str("1")  # green          Go back home
        led_srv(name_json)

        self.task_rotation(rad_90, pi * 1.5, 0.2)  # 0 deg counter clockwise, middle left turn
        self.task_straight_125cm(0.08)              # Go straight, center-left to bottom-left 
        self.task_rotation(pi, pi * 3, 0.3)      # Turn 180

        name_json.data = str("3")  # red
        led_srv(name_json)
        time.sleep(5)


        name_json.data = str("4") 
        led_srv(name_json)

        # self.task_rotation(rad_90, pi * 2.2, 1.2)
        msg_velocity = Twist2DStamped()
        msg_velocity.header.stamp = rospy.Time.now()
        msg_velocity.v = 0.35
        rate = rospy.Rate(20)
        for i in range(115):
            if i < 50:
                msg_velocity.omega = -pi/2.25
            else:
                msg_velocity.omega = -pi/1.55
            self.vel_pub.publish(msg_velocity)
            rate.sleep()
        
        for _ in range(10):
            self.stop(0.1)

        name_json.data = str("3") 
        led_srv(name_json)

        self.bag.close()

        rospy.loginfo("Closed the bag!")




    def update_frames(self, delta_rt, delta_lt):
        delta_rw_dist = (2 * pi * self.r * delta_rt) / 135
        delta_lw_dist = (2 * pi * self.r * delta_lt) / 135

        delta_dist_cover = (delta_rw_dist + delta_lw_dist)/2

        self.Xr += delta_dist_cover

        self.Th  += (delta_rw_dist - delta_lw_dist) / (2 * self.L)
        self.Xw += (delta_dist_cover * cos(self.Th)) 
        self.Yw += (delta_dist_cover * sin(self.Th))

        th = Float32()
        th.data = self.Th

        Xw = Float32()
        Xw.data = self.Xw

        Yw = Float32()
        Yw.data = self.Yw

        self.bag.write('th', th)
        self.bag.write('Xw', Xw)
        self.bag.write('Yw', Yw)


        return delta_dist_cover

    def task_rotation(self, rotation_amount, omega_amount, stopping_offset):
        rospy.loginfo("Starting rotation task..")
        rospy.loginfo(f'Initial Theta: {self.Th}')
        msg_velocity = Twist2DStamped()
        rate = rospy.Rate(50)

        initial_theta = self.Th
        
        target_theta = initial_theta + rotation_amount
        prv_rt = self.rt
        prv_lt = self.lt

        while True:
            if rotation_amount < 0 and self.Th - stopping_offset < target_theta:
                break
            if rotation_amount > 0 and self.Th + stopping_offset > target_theta:
                break

            delta_rt = self.rt - prv_rt
            delta_lt = self.lt - prv_lt

            prv_rt = self.rt
            prv_lt = self.lt

            msg_velocity.header.stamp = rospy.Time.now()
            msg_velocity.v = 0
            msg_velocity.omega = omega_amount

            self.update_frames(delta_rt, delta_lt)
            self.vel_pub.publish(msg_velocity)
            rospy.loginfo(f'Self.Th: {self.Th}, target_th: {target_theta}')
            rate.sleep()

        rospy.loginfo(f'Final Theta: {self.Th}')
        
        for i in range(10):
            self.stop(0.1)

    def task_straight_125cm(self, bias):
        rospy.loginfo("Starting 1.25m task..")

        msg_velocity = Twist2DStamped()
        rate = rospy.Rate(20)
        dist_cover = 0

        vel = 0.3
        prv_rt = self.rt
        prv_lt = self.lt

        start_th = self.Th

        while dist_cover <  1.1:
            delta_rt = self.rt - prv_rt
            delta_lt = self.lt - prv_lt

            prv_rt = self.rt
            prv_lt = self.lt

            dist_cover += self.update_frames(delta_rt, delta_lt)
            
            msg_velocity.header.stamp = rospy.Time.now()
            msg_velocity.v = vel
            theta_diff = self.Th - start_th
            msg_velocity.omega = (-theta_diff * 2.5) + bias
            
            self.vel_pub.publish(msg_velocity)
            rospy.loginfo(f'Xw: {self.Xw}, Yw: {self.Yw}, Th: {self.Th}, msg_velocity.omega: {msg_velocity.omega }')
            
            rate.sleep()

        rospy.loginfo("Distance covered: " + str(dist_cover))
        for i in range(10):
            self.stop(0.1)
        

    def task125cm(self):
        rospy.loginfo("Starting 1.25m task..")

        msg_velocity = Twist2DStamped()
        rate = rospy.Rate(20)
        dist_cover = 0

        vel = 0.5
        prv_rt = self.rt
        prv_lt = self.lt

        while dist_cover <  1.25:
            delta_rt = self.rt - prv_rt
            delta_lt = self.lt - prv_lt

            prv_rt = self.rt
            prv_lt = self.lt

            dist_cover += self.update_frames(delta_rt, delta_lt)
            
            msg_velocity.header.stamp = rospy.Time.now()
            msg_velocity.v = vel
            msg_velocity.omega = -self.Th * 1.5
            
            self.vel_pub.publish(msg_velocity)
            rospy.loginfo(f'Xw: {self.Xw}, Yw: {self.Yw}, Th: {self.Th}, dist: {dist_cover}, d_rt: {delta_rt}, d_lt: {delta_lt}')
            
            rate.sleep()

        rospy.loginfo("Distance covered: " + str(dist_cover))

        rospy.loginfo("Stopping")
        for i in range(10):
            self.stop(0.1)

        while dist_cover >  0:
            delta_rt = self.rt - prv_rt
            delta_lt = self.lt - prv_lt

            prv_rt = self.rt
            prv_lt = self.lt

            dist_cover += self.update_world_frame(delta_rt, delta_lt)
            
            msg_velocity.header.stamp = rospy.Time.now()
            msg_velocity.v = -vel
            msg_velocity.omega = -self.Th * 1.5
            
            self.vel_pub.publish(msg_velocity)
            rospy.loginfo(f'X: {self.X}, Y: {self.Y}, Th: {self.Th}, dist: {dist_cover}, d_rt: {delta_rt}, d_lt: {delta_lt}')
            
            rate.sleep()
        
        rospy.loginfo("Stopping")
        for i in range(10):
            self.stop(0.1)

    
    def led_test(self):
        rospy.loginfo("Waiting for led service..")
        rospy.wait_for_service('led_control_srv')
        rospy.loginfo("Connected to led service.")
        try:
            for i in range(4):
                led = rospy.ServiceProxy('led_control_srv', GetVariable)
                name_json = String()
                name_json.data = str(i + 1) 
                led(name_json)
                time.sleep(3)
                
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)



    def run(self):
        # self.task125cm()
        # self.task_rotation()
        # self.led_test()
        self.ex_two_part_two()


if __name__ == '__main__':
    # create the node
    node = DuckieMover(node_name='my_mover_node')
    node.run()
    rospy.loginfo("Done")
    # keep spinning
    # rospy.spin()