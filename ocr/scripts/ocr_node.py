'''********************************************************************
 * ocr_node.cpp
 *
 * Co-Author: Kwon Jin (School of Computing, KAIST)
 * Co-Author: Dongwon Choi (School of Computing, KAIST)
 *
 * Date (Last Modify):2023.06.15
 *
 * Detection the text contour and classification and the text by OCR
 *
 *******************************************************************'''
from transformers import TrOCRProcessor, VisionEncoderDecoderModel
from PIL import Image
import cv2
import torch
from difflib import SequenceMatcher
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from imutils.perspective import four_point_transform
from imutils.contours import sort_contours
import imutils
import time
import numpy as np
import pytesseract

class ocrNode :
  def __init__ (self) :
    # Init node name to 'ocr_node'
    rospy.init_node('ocr_node')

    '''******************************************************************
    * Notation)
    *   Node Name = NN , Message Type = MT , Callback Function = CF
    *
    * (Publishers)
    * pub_pos      :
    *   detected text contour position publisher
    *   (NN : '/ocr/place_pos')
    *
    * pub_tog      :
    *   start place toggle publisher
    *   (NN : '/place/toggle')
    *
    * pub2      :
    *   detected line mask image publisher (image_transport Publisher)
    *   (NN : "/finish_line_detect/line_img")
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
    * Subsciber2 :
    *   start OCR toggle topic subscriber
    *   (NN : '/ocr/toggle', CF : self.toggleCallback)
    *
    * Subsciber3 :
    *   classificated object name by clsassification node subscriber
    *   (NN : '/classification_node/obj_name', CF : self.nameCallback)
    *
    * Subsciber4 :
    *   reset toggle for place subscirber
    *   (NN : '/ocr/reset', CF : self.resetCallback)
    *
    * Subsciber5 :
    *   terminate toggle subscriber
    *   (NN : 'ocr/terminate', CF : self.terminateCallback)

    *
    '''
    self.pub_pos = rospy.Publisher('/ocr/place_pos', Pose2D, queue_size=1)
    self.pub_tog = rospy.Publisher('/place/toggle', String, queue_size=1)
    # self.pub_1 = rospy.Publisher('/ocr/recog', Image, queue_size=1)
    # self.pub_2 = rospy.Publisher('/ocr/recog2', Image, queue_size=1)
    # self.pub_3 = rospy.Publisher('/ocr/recog3', Image, queue_size=1)
    # self.pub_4 = rospy.Publisher('/ocr/recog4', Image, queue_size=1)
    rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
    rospy.Subscriber('/ocr/toggle', String, self.toggleCallback)
    # rospy.Subscriber('/object/name', String, self.objectCallback)
    rospy.Subscriber('/classification_node/obj_name', String, self.nameCallback)
    rospy.Subscriber('/ocr/reset', String, self.resetCallback)
    rospy.Subscriber('ocr/terminate', String, self.terminateCallback)


    self.edge_img = None
    self.obj_name = None
    self.ocr_toggle = 0
    self.bridge = CvBridge()
    self.pos_msg = Pose2D()
    self.pos_msg.x = 0
    self.pos_msg.y = 0
    self.det_success = 0
    self.reset_toggle = 0

  ''' terminateCallback function
  Parameters : self, (std_msgs.String) msg
  Return : void

  If subscribed msg.data is "Terminate",
  terminate node.
  '''
  def terminateCallback(self, msg) :
    if msg.data == "Terminate" :
      rospy.signal_shutdown("Success !")

  ''' resetCallback function
  Parameters : self, (std_msgs.String) msg
  Return : void

  If subscribed msg.data is "Rest",
  set det_success to 0 and reset_toggle to 1 for restart OCR
  '''
  def resetCallback(self, msg) :
    if msg.data == "Reset" :
      self.det_success = 0
      self.reset_toggle = 1

  ''' nameCallback function
  Parameters : self, (std_msgs.String) msg
  Return : void

  If subscribed msg.data is not None,
  set self.obj_name to msg.data

  object_name list
  ("cantata" , "tejava" , "oronamin" , "welchs", "demisoda")
  '''
  def nameCallback(self, msg):
    if msg.data != None :
      self.obj_name = msg.data

  ''' toggleCallback function
  Parameters : self, (std_msgs.String) msg
  Return : void

  If subscribed msg.data is "Start",
  set self.ocr_toggle to 1 to start OCR.
  '''
  def toggleCallback(self, msg) :
    if msg.data == "Start" :
      self.ocr_toggle = 1

  ''' make_scan_image function
  Parameters : self, image, image, ksize=(5,5), min_threshold=100, max_threshold=200, max_count=2, detail=False
  Return : transform_images, find_places

  Find the contour by using canny edge detection and ordering of big size contour.
  Then return the rectificated image and position of detected contour
  '''
  def make_scan_image(self, image, ksize=(5,5), min_threshold=100, max_threshold=200, max_count=2, detail=False):
    image_list_title = []
    image_list = []

    org_image = image.copy()
    ratio = org_image.shape[1] / float(image.shape[1])

    # Conver bgr image to hsv image and
    # remove noise by using gaussian blur
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv, ksize, 0)

    # White color filtering
    extracted = cv2.inRange(blurred, (0,0,255-100), (255,100,255))
    #extracted = cv2.inRange(blurred, (150,50,50), (190,255,255))

    # Canny Edge Detection
    edged = cv2.Canny(extracted, min_threshold, max_threshold)
    image_list_title = ['hsv', 'blurred', 'extracted', 'edged']
    image_list = [hsv, blurred, extracted, edged]

    # Find contours and choose by big size order
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)

    find_cnts, find_places, transform_images = [], [], []
    count = max_count
    output = image.copy()

    # Draw the polygon at the detected contour
    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)

        if len(approx) == 4:
            cv2.drawContours(output, [approx], -1, (0, 255, 0), 10)
            find_cnts.append(approx)
        if count - 1 <= 0:
            break
        else:
            count -= 1

    image_list_title.append("Outline")
    image_list.append(output)

    # for i in range(len(image_list)):
    #     plt_imshow(image_list_title[i], image_list[i])

    # Rectification by using perspective transformation with four points
    # at edge of the rectangle conour.
    for i,fc in enumerate(sorted(find_cnts, key=lambda x: np.min(x.reshape(4,2), axis=0)[0])):
        transform_image = four_point_transform(org_image, fc.reshape(4, 2) * ratio)
        transform_images.append(transform_image)
        find_places.append(np.mean(fc.reshape(4,2), axis=0).tolist())

    if detail:
      pass
      # # while True :
      # self.pub_1.publish(self.bridge.cv2_to_imgmsg(cv2.bilateralFilter(transform_images[0], -1, 10, 5), "bgr8"))
      # self.pub_2.publish(self.bridge.cv2_to_imgmsg(cv2.bilateralFilter(transform_images[1], -1, 10, 5), "bgr8"))
      # self.pub_3.publish(self.bridge.cv2_to_imgmsg(cv2.bilateralFilter(transform_images[2], -1, 10, 5), "bgr8"))
      # self.pub_4.publish(self.bridge.cv2_to_imgmsg(cv2.bilateralFilter(transform_images[3], -1, 10, 5), "bgr8"))

    return transform_images, find_places

  ''' process_image function
  Parameters : self, image
  Return : results

  Process with trained hand-written recognize model (trocr) and return
  the result
  '''
  def process_image(self, images):
    results = []
    for image in images:
      #image = cv2.bilateralFilter(image, -1, 10, 5)
      pixel_values = processor(image, return_tensors="pt").to(device).pixel_values
      generated_ids = model.to(device).generate(pixel_values, max_length = 20)
      generated_text = processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
      results.append(generated_text)

    return results

  # def process_image(self, images):
  #   results = []
  #   cnt = 1
  #   for image in images:
  #     generated_text = pytesseract.image_to_string(image, lang='eng')
  #     results.append(generated_text)
  #     cnt+=1

  #   return results

  ''' similar function
  Parameters : self, a, b
  Return : Similar Ratio

  Compute similarity of the given text argument
  '''
  def similar(self, a, b):
    return SequenceMatcher(None, a, b).ratio()

  ''' get_answer function
  Parameters : self, ground_truths, results
  Return : answers

  Compute simliarity and by using this return the answer that the
  closest word
  '''
  def get_answer(self, ground_truths, results):
    answers = []
    for word in results:
      similars = [self.similar(g, word.lower().strip()) for g in ground_truths]
      max_index = similars.index(max(similars))
      answers.append(ground_truths[max_index])

    return answers

  ''' trocr function
  Parameters : self, frame, ground_truth, cnt, detail
  Return : answers, detected_places

  Predit the word by using trocr with frame.
  Then return the answer list and detected position
  '''
  def trocr(self, frame, ground_truth, cnt, detail):
    image = frame.copy()

    detected_images, detected_places = self.make_scan_image(image, ksize=(5, 5), min_threshold=50, max_threshold=200, max_count=cnt, detail=detail)

    results = self.process_image(detected_images)
    answers = self.get_answer(ground_truth, results)

    rospy.loginfo("=====OCR Result=====")
    rospy.loginfo(results)
    rospy.loginfo("=====OCR Corrected Result=====")
    rospy.loginfo(answers)
    rospy.loginfo(detected_places)

    return answers, detected_places

  # def tesseract(self, path, ground_truth, cnt, detail):
  #   image = path.copy()

  #   detected_images, detected_places = self.make_scan_image(image, ksize=(5, 5), min_threshold=20, max_threshold=150, max_count=cnt, detail=detail)
  #   rospy.loginfo('Before : ' + str(len(detected_images)))
  #   results = self.process_image(detected_images)
  #   rospy.loginfo('After : ' + str(len(results)))
  #   answers = self.get_answer(ground_truth, results)
  #   rospy.loginfo('Ans : ' + str(len(answers)))
  #   print("=====Tesseract OCR Result=====")
  #   print(results)
  #   print("=====Tesseract OCR Corrected Result=====")
  #   print(answers)
  #   print(detected_places)

  #   return answers, detected_places

  ''' callback function
  Parameters : self, msg
  Return : void

  If ocr_toggle is 1, start OCR process with frame and publish the
  position of the text contour that object name.
  '''
  def callback(self, msg) :
    if self.ocr_toggle == 1:
      if self.det_success == 0 :
        try :
          rospy.loginfo("===OCR START===")
          # Convert message to cv image type
          frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
          # Preprocessing to raw_image from usb webcam (Flip)
          frame = cv2.flip(frame, -1)

          # OCR process
          ans, pos = self.trocr(frame, ["welchs", "cantata", "oronamin", "tejava", "demisoda"], 4, detail=True)

          if self.obj_name in ans :
            # Publish the poisition of the text contour of obj_name
            idx = ans.index(self.obj_name)
            self.pos_msg.x = pos[idx][0]
            self.pos_msg.y = pos[idx][1]
            self.pub_pos.publish(self.pos_msg)
            self.pub_tog.publish("Start")
            rospy.loginfo("[OCR] x : " + str(self.pos_msg.x))
            rospy.loginfo("[OCR] y : " + str(self.pos_msg.y))

            # If not on the reset_toggle, set det_success to 1 for
            # do not process OCR while reset_toggle is 1.
            if self.reset_toggle == 0:
              self.det_success = 1

        except Exception as e :
          rospy.logerr(e)

      # While reset_toggle is 1, just publish only the position of
      # the text contour of obj_name
      elif self.det_success == 1 :
        self.pub_pos.publish(self.pos_msg)



if __name__ == '__main__' :
  # Set device for use GPU if it is available
  device = "cuda:0" if torch.cuda.is_available() else "cpu"

  # processor = TrOCRProcessor.from_pretrained("microsoft/trocr-base-handwritten")
  # model = VisionEncoderDecoderModel.from_pretrained("microsoft/trocr-base-handwritten")

  # processor.save_pretrained("../models/trocr-base-handwritten")
  # model.save_pretrained("../models/trocr-base-handwritten")

  # Load the trained ocr model
  path = '/home/kj/catkin_ws/src/injiget_char/ocr/'
  processor = TrOCRProcessor.from_pretrained(path + "models/trocr-base-handwritten")
  model = VisionEncoderDecoderModel.from_pretrained(path + "models/trocr-base-handwritten")

  ocr_node = ocrNode()
  rospy.spin()
