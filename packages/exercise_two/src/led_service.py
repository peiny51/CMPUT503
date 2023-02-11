#!/usr/bin/env python3

from __future__ import print_function

from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, GetVariable
from std_msgs.msg import String
import rospy
import os

is_connected = False
service = None

def handle_state(req):
    global is_connected
    global service

    if not is_connected: 
        name = os.environ['VEHICLE_NAME']
        rospy.loginfo("Waiting for set_custom_pattern..")
        rospy.wait_for_service(f'/{name}/led_emitter_node/set_custom_pattern')
        rospy.loginfo("Connected to set_custom_pattern.")
        service = rospy.ServiceProxy(f'/{name}/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
        is_connected = True
    
    try:
        msg = LEDPattern()
        color1 = ['green', 'green', 'green', 'green', 'green']
        color2 = ['blue', 'blue', 'blue', 'blue', 'blue']
        color3 = ['red', 'red', 'red', 'red', 'red']
        color4 = ['purple', 'purple', 'purple', 'purple', 'purple']
        color5 = ['cyan', 'cyan', 'cyan', 'cyan', 'cyan']
        color6 = ['switchedoff', 'switchedoff', 'switchedoff', 'switchedoff', 'switchedoff']
        msg.color_mask = [1, 1, 1, 1, 1]
        msg.frequency = 0
        msg.frequency_mask = [1, 1, 1, 1, 1]

        state = int(req.name_json.data)
        
        if state == 1:
            msg.color_list = color1
            rospy.loginfo("Making leds green!")
        elif state == 2:
            msg.color_list = color2
            rospy.loginfo("Making leds blue!")
        elif state == 3:
            msg.color_list = color3
            rospy.loginfo("Making leds red!")
        elif state == 4:
            msg.color_list = color4
            rospy.loginfo("Making leds purple!")
        elif state == 5:
            msg.color_list = color5
            rospy.loginfo("Making leds cyan!")
        elif state == 6:
            msg.color_list = color6
            rospy.loginfo("Switching off leds!")
        else:
            pass
        
        service(msg)
        value_json = String()
        value_json.data = req.name_json.data
        return value_json

        
    except rospy.ServiceException:
        print("Service call failed")

def led_server():
    rospy.init_node('led_server')
    s = rospy.Service('led_control_srv', GetVariable, handle_state)
    rospy.spin()

if __name__ == "__main__":
    led_server()