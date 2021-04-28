#!/usr/bin/env python3

import cv2
import sys
import rospy
import message_filters

import numpy as np
import tflite_runtime.interpreter as tflite

from time import time
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2

# TODO: Figure out this path programmatically
MODEL_PATH = "/home/joey/catkin/src/bartholomew/tf_models/ssd_mobilenetv1/ssd_mobilenet_v1_1_metadata_1.tflite"

class Vision:
    def __init__(self):
        self.image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)

        self.heading_pub = rospy.Publisher("/bartholomew/object_heading", Float, 1)

        self.time_sync = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 1)
        self.time_sync.registerCallback(self.callback)

        self.bridge = CvBridge()

        # Instantiate tflite model: CenterNet MobileNetV2 FPN 512x512
        self.cnn = tflite.Interpreter(model_path=MODEL_PATH)
        self.cnn.allocate_tensors()

        self.input_details = self.cnn.get_input_details()
        self.output_details = self.cnn.get_output_details()

        self.input_shape = self.input_details[0]['shape']

    def draw_box(self, image, box):
        sx, sy = self.input_shape[1], self.input_shape[2]

        topleft = (int(sy*box[1]), int(sx*box[0]))
        bottomright = (int(sy*box[3]), int(sx*box[2]))
        cv2.rectangle(image, topleft, bottomright, (0, 255, 0), 3)

        return image

    # From fractional units to coordinates scaled to the input size
    # output == [y0, x0, y1, x1]
    def scale_box_coordinates(self, box):
        sx, sy = self.input_shape[1], self.input_shape[2]
        return [sy*box[0], sx*box[1], sy*box[2], sx*box[3]]

    # Crawl out in the cardinal directions for a valid point (i.e. not isnan(point))
    def find_nearest_point(self, start, box, cloud):
        limit = 1000

        points = [start.copy(), start.copy(), start.copy()] # NESW
        for i in range(limit):
            for p in range(len(points)):
                point = points[p]
                test = cloud[point[1]][point[0]]
                if not np.isnan(test):
                    return point

                if p == 0 and points[p][1] > box[0]:
                    points[p][1] -= 1 # North
                elif p == 1 and points[p][0] < box[3]:
                    points[p][0] += 1 # East
                elif points[p][0] > box[1]:
                    points[p][0] -= 1 # West
                else:
                    break

        print("Panic! No valid points found.")

        return [-1, -1] # Hopefully we don't reach this

    def callback(self, rgb_data, depth_data):
        t1 = time()
        try:
            image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")

            og_image_shape = image.shape

            image = cv2.resize(image, dsize=(self.input_shape[1], self.input_shape[2]), interpolation=cv2.INTER_CUBIC)
            image = image[np.newaxis,:]

            depth = self.bridge.imgmsg_to_cv2(depth_data)
            depth = cv2.resize(depth, dsize=(self.input_shape[1], self.input_shape[2]), interpolation=cv2.INTER_CUBIC)

        except CvBridgeError as e:
            print(e)
            return

        self.cnn.set_tensor(self.input_details[0]['index'], image)
        self.cnn.invoke()
        boxes = self.cnn.get_tensor(self.output_details[0]['index'])
        classes = self.cnn.get_tensor(self.output_details[1]['index'])
        class_probs = self.cnn.get_tensor(self.output_details[2]['index'])

        where_human = np.where(np.logical_and(classes == 0, class_probs > 0.2)) # 0 is the ID for person

        filtered_boxes = boxes[where_human]
        filtered_probs = class_probs[where_human]

        object_heading = None

        if filtered_boxes.shape[0] > 0:
            best_box = filtered_boxes[np.argmax(filtered_probs)]

            #image = self.draw_box(image[0], best_box)
            #cv2.imshow("display", depth)
            #cv2.waitKey(3)

            centroid = self.scale_box_coordinates(best_box)
            centroid = [int((centroid[1]+centroid[3]) / 2), int((centroid[0]+centroid[2]) / 2)]

            z = depth[centroid[1]][centroid[0]]
            if np.isnan(z):
                candidate = self.find_nearest_point(centroid, self.scale_box_coordinates(best_box), depth)
                if (candidate[0] != -1) and (candidate[1] != -1):
                    centroid = candidate
                    z = depth[centroid[1]][centroid[0]]
                else:
                    # 6 meters is the max perception distance of the depth sensors
                    z = 6

            xfocal_length = 530 * (self.input_shape[2] / og_image_shape[1])
            yfocal_length = 530 * (self.input_shape[1] / og_image_shape[0])

            cx = self.input_shape[1] / 2
            cy = self.input_shape[2] / 2

            xw = z * (centroid[0]-cx) / xfocal_length
            yw = z * (centroid[1]-cy) / yfocal_length

            # object_heading is relative to the current camera heading
            #opposite = centroid[0] - depth.shape[1]//2
            object_heading = np.arctan2(xw, z) * 180 / 3.14159

            t2 = time()

            self.heading_pub.publish(object_heading)

            print(f"t2 - t1: {t2-t1} -> object_location: {(xw, yw, z, object_heading)}")#{object_heading}")

def main(args):
    vision = Vision()
    print("Class initialized")
    rospy.init_node('vision', anonymous=True)
    print("Node initialized")

    try:
        print("Spinning")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down vision")

if __name__ == "__main__":
    main(sys.argv)
