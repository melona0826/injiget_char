'''********************************************************************
 * ocr_node.cpp
 *
 * Co-Author: Kwon Jin (School of Computing, KAIST)
 * Co-Author: Dongwon Choi (School of Computing, KAIST)
 * Co-Author : Youngwoo Jung (Department of Chemistry)
 *
 * Date (Last Modify):2023.06.15
 *
 * Detection the text contour and classification and the text by OCR
 *
 *******************************************************************'''
import torch
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from ultralytics import YOLO

# Object lists that classfication
objects = {0:"cantata" , 1:  "tejava", 2:  "oronamin", 3: "welchs", 4: "demisoda"}

class ClassficationNode() :
  def __init__(self) :
    # Init node name to 'classification_node'
    rospy.init_node('classification_node')

    '''******************************************************************
    * Notation)
    *   Node Name = NN , Message Type = MT , Callback Function = CF
    *
    * (Publishers)
    * pub      :
    *   predicted object name publisher
    *   (NN : '/classification_node/obj_name')
    *
    * pub_pick_start      :
    *   start pick toggle publisher
    *   (NN : '/pallet_det/toggle')
    *
    * pub_box_img      :
    *   detected box image publisher
    *   (NN : '/classification_node/box_img'")
    *
    * (Subscribers)
    * (*** In ocr_node, subscriber does not have a name ***)
    * (In this comment block, we give a name with number just for
    *  distinguish)
    *
    * Subsciber1 :
    *   raw_image from the usb web cam subscriber
    *   (NN : '/usb_cam/image_raw' , CF : self.callback)
    *
    '''
    self.pub = rospy.Publisher('/classification_node/obj_name', String, queue_size=1)
    self.pub_pick_start = rospy.Publisher('/pallet_det/toggle', String, queue_size=1)
    self.pub_box_img = rospy.Publisher('/classification_node/box_img', Image, queue_size=1)
    rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)

    self.bridge = CvBridge()
    self.model = YOLO("/home/kj/catkin_ws/src/injiget_char/classification/model/best.pt")
    self.toggle = 1

    self.obj_name = None
    self.box_img = None

  ''' callback function
  Parameters : self, msg
  Return : void

  Classfication the object from the frame image and
  classificated publish the object name
  '''
  def callback(self, msg) :
    if self.toggle == 1 :
      # Convert image message to cv image type bgr8
      frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

      # Preprocess (Flip)
      frame = cv2.flip(frame, -1)

      # Classification with frame by model
      result = self.model(frame)
      self.box_img = result[0].plot()

      label = int(result[0].boxes.cls[0].item())
      prob = result[0].boxes.conf[0].item()
      rospy.loginfo(label)

      # Set self.obj_name to the highst probability predicted object name
      self.obj_name = objects[label]

      if self.obj_name != None :
        # Set toggle to 0 for classification only once.
        self.toggle = 0
        rospy.loginfo("Detect object : " + str(self.obj_name))
        time.sleep(1)

        # Publish start toggle of pick to "Start" ,
        # classificated object name,
        # detected box image.
        self.pub_pick_start.publish("Start")
        self.pub.publish(self.obj_name)
        self.pub_box_img.publish(self.bridge.cv2_to_imgmsg(self.box_img, "bgr8"))

    # Publish the classficated obj_name
    self.pub.publish(self.obj_name)



if __name__ == '__main__' :
  classification_node = ClassficationNode()
  rospy.spin()
