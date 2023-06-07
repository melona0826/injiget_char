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
    rospy.init_node('ocr_node')
    self.pub_pos = rospy.Publisher('/ocr/place_pos', Pose2D, queue_size=1)
    self.pub_tog = rospy.Publisher('/place/toggle', String, queue_size=1)
    # self.pub_1 = rospy.Publisher('/ocr/recog', Image, queue_size=1)
    # self.pub_2 = rospy.Publisher('/ocr/recog2', Image, queue_size=1)
    # self.pub_3 = rospy.Publisher('/ocr/recog3', Image, queue_size=1)
    # self.pub_4 = rospy.Publisher('/ocr/recog4', Image, queue_size=1)
    self.edge_img = None

    self.ocr_toggle = 0
    self.bridge = CvBridge()
    self.pos_msg = Pose2D()
    self.pos_msg.x = 0
    self.pos_msg.y = 0

    rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
    rospy.Subscriber('/ocr/toggle', String, self.toggleCallback)
    rospy.Subscriber('/object/name', String, self.objectCallback)

  def make_scan_image(self, image, ksize=(5,5), min_threshold=100, max_threshold=200, max_count=2, detail=False):
    image_list_title = []
    image_list = []

    org_image = image.copy()
    ratio = org_image.shape[1] / float(image.shape[1])

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(gray, ksize, 0)
    extracted = cv2.inRange(blurred, (0,0,255-100), (255,100,255))
    #   extracted = cv2.inRange(blurred, (150,50,50), (190,255,255))
    edged = cv2.Canny(extracted, min_threshold, max_threshold)
    image_list_title = ['gray', 'blurred', 'extracted', 'edged']
    image_list = [gray, blurred, extracted, edged]

    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)

    find_cnts, find_places, transform_images = [], [], []
    count = max_count
    output = image.copy()
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
  #     rospy.logerr('SIBAL! ' + str(cnt))
  #     generated_text = pytesseract.image_to_string(image, lang='eng')
  #     results.append(generated_text)
  #     cnt+=1

  #   return results

  def similar(self, a, b):
    return SequenceMatcher(None, a, b).ratio()

  def get_answer(self, ground_truths, results):
    answers = []
    for word in results:
      similars = [self.similar(g, word.lower().strip()) for g in ground_truths]
      max_index = similars.index(max(similars))
      answers.append(ground_truths[max_index])

    return answers

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

  def toggleCallback(self, msg) :
    if msg.data == "OCR" :
      self.ocr_toggle = 1
    else :
      self.ocr_toggle = 0

  def objectCallback(self, msg):
    self.obj_name = msg.data

  def callback(self, msg) :
    if self.ocr_toggle == 1 :
      try :
        rospy.loginfo("===OCR START===")
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = cv2.flip(frame, -1)

        ans, pos = self.trocr(frame, ["welchs", "cantata", "oronamin", "tejava", "demisoda"], 4, detail=True)

        idx = ans.index(self.obj_name)
        self.pos_msg.x = pos[idx][0]
        self.pos_msg.y = pos[idx][1]
        self.pub_pos.publish(self.pos_msg)
        self.pub_tog.publish("Place")
        rospy.loginfo("[OCR] x : " + str(self.pos_msg.x))
        rospy.loginfo("[OCR] y : " + str(self.pos_msg.y))
        time.sleep(1)
        #rospy.signal_shutdown("Success OCR")



      except Exception as e :
        rospy.logerr(e)



if __name__ == '__main__' :
  device = "cuda:0" if torch.cuda.is_available() else "cpu"

  # processor = TrOCRProcessor.from_pretrained("microsoft/trocr-base-handwritten")
  # model = VisionEncoderDecoderModel.from_pretrained("microsoft/trocr-base-handwritten")

  # processor.save_pretrained("../models/trocr-base-handwritten")
  # model.save_pretrained("../models/trocr-base-handwritten")
  path = '/home/kj/catkin_ws/src/injiget_char/ocr/'
  processor = TrOCRProcessor.from_pretrained(path + "models/trocr-base-handwritten")
  model = VisionEncoderDecoderModel.from_pretrained(path + "models/trocr-base-handwritten")

  ocr_node = ocrNode()
  rospy.spin()
