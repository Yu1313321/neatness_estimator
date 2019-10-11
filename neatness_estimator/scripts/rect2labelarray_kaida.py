#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy

import message_filters
from jsk_recognition_msgs.msg import Rect, RectArray
from jsk_recognition_msgs.msg import ClassificationResult

try:
    from rect_projector_msgs.msg import Scored2DBox, Scored2DBoxArray
except:
    rospy.logerr("please install aero-ros-pkg-private")
    exit()


class Rect2LabeledArray():

    def __init__(self):
        self.pub = rospy.Publisher('/kaida/rect/boxes', Scored2DBoxArray, queue_size=1)
        self.subscribe()

    def subscribe(self):
        queue_size = rospy.get_param('/kaida/rect/queue', 100)

        sub_box = message_filters.Subscriber(
            '/mask_rcnn_instance_segmentation/output/rects', RectArray, queue_size=queue_size)
        sub_class = message_filters.Subscriber(
            '/mask_rcnn_instance_segmentation/output/class', ClassificationResult, queue_size=queue_size)

        self.subs = [sub_box, sub_class]
        sync = message_filters.TimeSynchronizer(
            fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.printMsg)

    def printMsg(self, boxes_msg, classes_msg):
        for rect in boxes_msg.rects:
            print(rect.x)
            print(rect.y)


if __name__ == '__main__':
    rospy.init_node("rect_to_labeledarray")
    r2la = Rect2LabeledArray()
    rospy.spin()
