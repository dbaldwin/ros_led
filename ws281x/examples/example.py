# sudo apt-get install python3-rospy

import rclpy
from rclpy.node import Node
from led_msgs.srv import SetLeds
from led_msgs.msg import LedStateArray, LedState

# led_count = get_parameter("/led/led_count")
# print(led_count)



create_client(SetLeds, 'led/set_leds')

#set_leds = rospy.ServiceProxy('led/set_leds', SetLEDs, persistent=True)

# def fill_strip(red, green, blue):
#     print(set_leds(leds=[LedState(i, red, green, blue) for i in range(led_count)]))

# fill_strip(200, 200, 200)
# rospy.sleep(1)
# fill_strip(0, 200, 0)
# rospy.sleep(1)
# fill_strip(0, 0, 200)
# rospy.sleep(1)
# fill_strip(200, 0, 0)
# rospy.sleep(1)
# fill_strip(0, 0, 0)
