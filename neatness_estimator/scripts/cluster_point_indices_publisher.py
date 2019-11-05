#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy

import message_filters
from jsk_recognition_msgs.msg import Rect, RectArray
from jsk_recognition_msgs.msg import ClusterPointIndices
from pcl_msgs.msg import PointIndices
from sensor_msgs.msg import Image

def one_shot_subscribe(topic_name, mclass=None,
                       timeout=None, after_stamp=None,
                       condition=None,
                       frequency=10):
    """
    Subscribe message, just for once
    """
    if mclass is None:
        import rostopic
        import importlib
        import rosgraph
        master = rosgraph.masterapi.Master('/rostopic')
        topic_types = dict(rostopic._master_get_topic_types(master))
        topic_type = topic_types[topic_name].split('/')
        pkg = importlib.import_module('{}.msg'.format(topic_type[0]))
        mclass = getattr(pkg, topic_type[1])

    if after_stamp is not None and isinstance(after_stamp, rospy.Time):
        raise TypeError('after_stamp should be rospy.Time, but get {}'
                        .format(type(after_stamp)))

    class OneShotSubscriber(object):

        def __init__(self):
            self.msg = None
            self.sub = rospy.Subscriber(
                topic_name, mclass,
                self.one_shot_subscribe)

        def unsubscribe(self):
            self.sub.unregister()

        def one_shot_subscribe(self, msg):

            if after_stamp is not None:
                if msg.header.stamp.to_sec() > after_stamp.to_sec():
                    self.msg = msg
            else:
                self.msg = msg

    oss = OneShotSubscriber()

    finish_time = None
    if timeout is not None:
        finish_time = rospy.Time.now()
    if finish_time is not None:
        finish_time = finish_time + rospy.Duration(timeout / 1000.0)

    r = rospy.Rate(frequency)
    while not rospy.is_shutdown() and \
        not (finish_time is not None and
             (finish_time - rospy.Time.now()).to_sec < 0.0):
        r.sleep()
        if oss.msg is not None:
            if (condition is not None):
                if callable(condition) and \
                   condition(oss.msg):
                    break
            else:
                break
    oss.unsubscribe()
    return oss.msg



class ClusterPointIndicesPublisher():
    def __init__(self):
        self.pub_indices = rospy.Publisher("~output", ClusterPointIndices, queue_size=1)
        self.image = one_shot_subscribe("~input_image", Image)
        sub_rect = rospy.Subscriber("~input_rect", RectArray, self.indices_publisher)

    def indices_publisher(self, msg):
        msg_indices = ClusterPointIndices(header=msg.header)
        for rect in msg.rects:
            H = self.image.height
            W = self.image.width
            bbox = [rect.y, rect.x, rect.height+rect.y, rect.width+rect.x]
            indices = np.arange(H * W).reshape(H, W)[bbox[0]:bbox[2],bbox[1]:bbox[3]].reshape(-1)
            indices_msg = PointIndices(header=msg.header, indices=indices)
            msg_indices.cluster_indices.append(indices_msg)
        self.pub_indices.publish(msg_indices)


if __name__ == "__main__":
    rospy.init_node("cluster_points_indices_publisher")
    indices = ClusterPointIndicesPublisher()
    rospy.spin()
