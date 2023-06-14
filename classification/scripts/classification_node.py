import torch
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from ultralytics import YOLO

objects = {0:"cantata" , 1:  "tejava", 2:  "oronamin", 3: "welchs", 4: "demisoda"}

class ClassficationNode() :
  def __init__(self) :
    rospy.init_node('classification_node')
    self.pub = rospy.Publisher('/classification_node/obj_name', String, queue_size=1)
    self.pub_pick_start = rospy.Publisher('/pallet_det/toggle', String, queue_size=1)
    self.pub_box_img = rospy.Publisher('/classification_node/box_img', Image, queue_size=1)
    self.bridge = CvBridge()
    self.model = YOLO("/home/kj/catkin_ws/src/injiget_char/classification/model/best.pt")
    self.toggle = 1
    self.obj_name = None
    self.box_img = None
    pub_tilt = rospy.Publisher('/tilt/mode', String, queue_size=1)
    pub_fork = rospy.Publisher('/fork/mode', String, queue_size=1)

    pub_fork.publish("down")
    pub_tilt.publish("front")

    time.sleep(1)
    rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)

  def callback(self, msg) :
    if self.toggle == 1 :
      frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
      frame = cv2.flip(frame, -1)
      result = self.model(frame)
      self.box_img = result[0].plot()

      label = int(result[0].boxes.cls[0].item())
      prob = result[0].boxes.conf[0].item()
      rospy.loginfo(label)
      self.obj_name = objects[label]
      if self.obj_name != None :
        self.toggle = 0
        rospy.loginfo("Detect object : " + str(self.obj_name))
        time.sleep(1)
        self.pub_pick_start.publish("Start")
        self.pub.publish(self.obj_name)
        self.pub_box_img.publish(self.bridge.cv2_to_imgmsg(self.box_img, "bgr8"))
        # time.sleep(1)
        # rospy.signal_shutdown("Success Classification")

    self.pub.publish(self.obj_name)



if __name__ == '__main__' :
  classification_node = ClassficationNode()
  rospy.spin()
