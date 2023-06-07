from ultralytics import YOLO
import torch
import cv2
import numpy as np
import pathlib
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import String
from sensor_msgs import Image
from cv_bridge import CvBridge, CvBridgeError
import time

objects = {0:"cantata" , 1:  "tajava", 2:  "oronamic", 3: "weltchs", 4: "demisoda"}

class ClassficationNode() :
  def __init__(self) :
    rospy.init_node('yolo_node')
    self.pub = rospy.Publisher('/yolo_node/obj_name', String, queue_size=1)
    self.bridge = CvBridge()
    rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
    self.mdoel = YOLO('/home/kj/catkin_ws/src/injiget_char/classification/model/best.pt')
  def callback(self, msg) :
    frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    frame = cv2.flip(frame, -1)
    result = model(frame)

    label = int(results[0].boxes.cls[0].item())
    prob = results[0].boxes.conf[0].item()

    obj_name = objects[label]

    pub.publish(obj_name)



if __name__ == '__main__' :
  pub_tilt = rospy.Publisher('/tilt/mode', String, queue_size=1)
  pub_fork = rospy.Publisher('/fork/mode', String, queue_size=1)

  pub_fork.publish("down")
  pub_tilt.publish("front")

  time.sleep(1)

  classification_node = ClassficationNode()
  rospy.spin()
