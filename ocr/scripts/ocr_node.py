from transformers import TrOCRProcessor, VisionEncoderDecoderModel
from PIL import Image
import cv2
import image_preprocess
import torch
from difflib import SequenceMatcher
import rospy
from sensor_msgs import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs import Pose2D
from std_msgs import String
import time

class ocrNode :
  def __init__ (self) :
    rospy.init_node('ocr_node')
    self.pub_pos = rospy.Publisher('/ocr/place_pos', Pose2D, queue_size=1)
    self.pub_tog = rospy.Publisher('/place/toggle', String, queue_size=1)
    self.pos_msg
    self.ocr_toggle = 0
    self.bridge = CvBridge()
    self.obj_name
    rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
    rospy.Subscriber('/ocr/toggle', String, self.toggleCallback)
    rospy.Subscriber('/object/name', String, self.objectCallback)

  def process_image(self, images):
    results = []
    for image in images:
      pixel_values = processor(image, return_tensors="pt").to(device).pixel_values
      generated_ids = model.to(device).generate(pixel_values, max_length = 20)
      generated_text = processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
      results.append(generated_text)

    return results

  def similar(self, a, b):
    return SequenceMatcher(None, a, b).ratio()

  def get_answer(self, ground_truths, results):
    answers = []
    for word in results:
      similars = [self.similar(g, word.lower().strip()) for g in ground_truths]
      max_index = similars.index(max(self.similars))
      answers.append(ground_truths[max_index])

    return answers

  def trocr(self, frame, ground_truth, cnt, detail):
    image = frame.copy()

    detected_images, detected_places = image_preprocess.make_scan_image(image, ksize=(5, 5), min_threshold=50, max_threshold=200, max_count=cnt, detail=detail)
    results = self.process_image(detected_images)
    answers = self.get_answer(ground_truth, results)
    rospy.loginfo("=====OCR Result=====")
    rospy.loginfo(results)
    rospy.loginfo("=====OCR Corrected Result=====")
    rospy.loginfo(answers)
    rospy.loginfo(detected_places)

    return answers, detected_places

  def toggleCallback(self, msg) :
    if msg.data == "OCR" :
      self.ocr_toggle = 1
    else :
      self.ocr_toggle = 0

  def objectCallback(self, msg):
    self.obj_name = msg.data

  def callback(self, msg) :
    if ocr_toggle == 1 :
      try :
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        ans, pos = self.trocr(frame, ground_truth, ["welchs", "cantata", "oronamin", "tejava", "demisoda"], 4, detail=True)

        idx = ans.index(self.obj_name)
        self.pos_msg.x = pos[idx][0]
        self.pos_msg.y = pos[idx][1]
        self.pub_pos(pos_msg)
        self.pub_tog("Place")

        time.sleep(1)
        rospy.signal_shutdown("Success OCR")



      except Exception as e :
        rospy.logerr(e)



if __name__ == '__main__' :
  device = "cuda:0" if torch.cuda.is_available() else "cpu"

  processor = TrOCRProcessor.from_pretrained("microsoft/trocr-base-handwritten")
  model = VisionEncoderDecoderModel.from_pretrained("microsoft/trocr-base-handwritten")

  processor.save_pretrained("../models/trocr-base-handwritten")
  model.save_pretrained("../models/trocr-base-handwritten")

  ocr_node = ocrNode()
  rospy.spin()
