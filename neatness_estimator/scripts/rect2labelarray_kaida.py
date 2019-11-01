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

    rect_x_list = []
    rect_y_list = []

    def __init__(self):
        self.subscribe()
        rospy.Timer(rospy.Duration(1), self.callbackTimer)

    def subscribe(self):
        queue_size = rospy.get_param('/kaida/rect/queue', 100)

        sub_box = message_filters.Subscriber(
            '/ssd_donbe_detector/output/rects', RectArray, queue_size=queue_size)
        sub_class = message_filters.Subscriber(
            '/ssd_donbe_detector/output/class', ClassificationResult, queue_size=queue_size)

        self.subs = [sub_box, sub_class]
        sync = message_filters.TimeSynchronizer(
            fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.printMsg)

    def printMsg(self, boxes_msg, labels_msg):
        print(boxes_msg)
        print(labels_msg.label_names)
        for rect, label in zip(boxes_msg.rects, labels_msg.label_names):
            print(label)
            if(label.label == "donbe"):
                self.rect_x_list.append(rect.x)
                self.rect_y_list.append(rect.y)

    def callbackTimer(self, event):
        print(self.rect_x_list)
        print(self.rect_y_list)
        print(np.var(np.array(self.rect_x_list)) + np.var(np.array(self.rect_y_list)))
        print("--------")
        del self.rect_x_list[:]
        del self.rect_y_list[:]

if __name__ == '__main__':
    rospy.init_node("rect_to_labeledarray")
    r2la = Rect2LabeledArray()
    rospy.spin()
