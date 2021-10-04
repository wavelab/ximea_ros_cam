#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

class JoyTrigger:
    def __init__(self):
        rospy.init_node('joy_trigger')
        self._joy_sub = rospy.Subscriber('/joy', Joy, self.joyCb)
        self._trigger_pub = rospy.Publisher('/camera/save_image', Empty, queue_size=1)

    def joyCb(self, joy_msg):
        button_press = joy_msg.buttons[0] # use "A" button on joystick
        if button_press:
            empty_msg = Empty()
            self._trigger_pub.publish(empty_msg)

if __name__ == '__main__':
    joy_trigger = JoyTrigger()
    rospy.spin()
