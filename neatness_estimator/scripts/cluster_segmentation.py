#!/usr/bin/env python
from __future__ import print_function

import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import message_filters
from jsk_recognition_msgs.msg import Rect, RectArray
from jsk_recognition_msgs.msg import LabelArray

try:
    from rect_projector_msgs.msg import Scored2DBox, Scored2DBoxArray
except:
    rospy.logerr("please install aero-ros-pkg-private")
    exit()

class image_converter:

    def __init__(self):
        self.subscribe()
        self.bridge = CvBridge()

    def subscribe(self):
        queue_size = rospy.get_param('/kaida/rect/queue', 100)

        rcnn_image_sub = message_filters.Subscriber(
            "/mask_rcnn_instance_segmentation/output/label_ins", Image, queue_size=queue_size)
        dec_image_sub = message_filters.Subscriber(
            "/instance_cluster_point_indices_decomposer/mask", Image, queue_size=queue_size)

        self.subs = [rcnn_image_sub, dec_image_sub]
        sync = message_filters.TimeSynchronizer(
            fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.printMsg)

    def printMsg(self, rcnn_image, dec_image):
        cv_rcnn_image = self.bridge.imgmsg_to_cv2(rcnn_image, "32SC1")
        cv_dec_image = self.bridge.imgmsg_to_cv2(dec_image, "mono8")
#        print(cv_rcnn_image.shape)
        debug_rcnn_image = np.zeros(cv_rcnn_image.shape)
        debug_dec_image = np.zeros(cv_dec_image.shape)
        debug_rcnn_image[:,:][cv_rcnn_image == 0] = 255
        debug_dec_image[:,:][cv_dec_image == 0] = 255

        print("--")
        #cv2.imshow("debug", debug_rcnn_image * debug_dec_image)
        #cv2.waitKey(100)


#  def __init__(self):
#    self.image_pub = rospy.Publisher("image_topic_2",Image)
#
#    self.bridge = CvBridge()
#    self.rcnn_image_sub = rospy.Subscriber("/mask_rcnn_instance_segmentation/output/label_ins",Image,self.callback)
#    self.dec_image_sub = rospy.Subscriber("/instance_cluster_point_indices_decomposer/mask",Image,self.callback)
#
#  def callback(self,data):
#    try:
#      cv_image = self.bridge.imgmsg_to_cv2(data, "32SC1")
#    except CvBridgeError as e:
#      print(e)
#
#
#    debug_img = np.zeros(cv_image.shape, dtype=np.uint8)
#    print(debug_img.shape)
#    print([cv_image == 1])

    # for y in cv_image.shape[0]:
    #   for x in cv_image.shape[1]:
    #     if cv_image[y][x] = 1:
    #       debug_img[y][x] = 255

    #for index in cv_image.max():
    #  debug_img[:,:][cv_image == index] = 255
    #  process and 

#    cv2.imshow("debug", debug_img)
#    cv2.waitKey(10)

#    (rows,cols,channels) = cv_image.shape
#    if cols > 60 and rows > 60 :
#      cv2.circle(cv_image, (50,50), 10, 255)
#
#    cv2.imshow("Image window", cv_image)
#    cv2.waitKey(3)
#
#    try:
#      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "32SC1"))
#    except CvBridgeError as e:
#      print(e)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
