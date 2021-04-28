#!/usr/bin/env python

import sys
import move
import rospy
from std_msgs.msg import Float32
from gyroscop import Gyroscope

LINEAR_SPEED = 40
ANGULAR_SPEED = 50

class MotorControl:
    def __init__(self):
        #self.heading_sub = rospy.Subscriber("self_heading", Float32, self.heading_callback)
        #self.object_sub = rospy.Subscriber("

        self.compass = Gyroscope()

    def produce_diff(self, new_heading):
        return new_heading - self.compass.get_reading()

    def correct_heading(self, new_heading):
        diff = self.produce_diff(new_heading)

        # Turn left
        if diff < 0:
            move.spinLeft(ANGULAR_SPEED)
            while abs(self.produce_diff(new_heading)) > 5:
                pass

            move.stop()
        elif diff > 0:
            move.spinRight(ANGULAR_SPEED)
            while abs(self.produce_diff(new_heading)) > 5:
                pass

            move.stop()

        return

def main(args):
    rospy.init_node("motor_control", anonymous=True)
    mc = MotorControl()

    try:
        print("spinning")
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

if __name__ == "__main__":
    main(sys.argv)
