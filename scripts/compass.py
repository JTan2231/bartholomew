#!/usr/bin/env python

import sys
import rospy

from std_msgs.msg import Float32
from gyroscope import Gyroscope

class Compass:
    def __init__(self):
        self.heading_pub = rospy.Publisher("self_heading", Float32, queue_size=10)

        self.gyroscope = Gyroscope()

    def publish_heading(self):
        print("Publishing")
        rate = rospy.Rate(5) # 10hz
        while not rospy.is_shutdown():
            reading = Float32(data=self.gyroscope.get_reading())
            #rospy.loginfo(reading)
            self.heading_pub.publish(reading)
            rate.sleep()

def main(args):
    compass = Compass()
    rospy.init_node('compass', anonymous=True)

    try:
        compass.publish_heading()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main(sys.argv)
