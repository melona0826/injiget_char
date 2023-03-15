import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import numpy as np

frame_wid = 640
frame_hei = 480

roi_x = 80
roi_y = 240
roi_width = frame_wid - roi_x
roi_height = frame_hei - roi_y

black_hue_low = 0
black_hue_high = 255
black_sat_low = 0
black_sat_high = 60
black_val_low = 0
black_val_high = 100

class Detect(self) :
  def __init__ (self):
    self.det_img = None
    self.line_img = None

    self.br = CvBridge()
    self.loop_rate = rospy.Rate(10)

    self.pub_det_img = rospy.Publisher('line_det/det_img', Image, queue_size=1)
    self.pub_line_img = rospy.Publisher('line_det/line_img', Image, queue_size=1)

    self.Subscriber('cam_pub/raw_img', Image, self.callback)


  def callback(self, msg) :
    frame = self.br.imgmsg_to_cv2(msg)



if __name__ == '__main__' :
  try :
    rospy.init_node('line_detect', anonymous=True)
    det_node = Detect()
    det_node.detect()

  except rospy.ROSInterruptException :
    pass
