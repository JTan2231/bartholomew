# Bartholomew
Selection of Bartholomew algorithms. Requires [TensorFlow Lite Runtime](https://www.tensorflow.org/lite/guide/python) version `2.5.0` and at least
Python 3.

Currently, the only algorithm in the repository is his vision capabilities.

# Vision
Locates a human being in Bartholomew's field of view and calculates the location of the person and the angle between himself and that person. Uses a
pretrained object detection model from [TensorFlow's Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md).
