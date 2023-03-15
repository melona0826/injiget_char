import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import numpy as np

class LineDetectnode() :
  def __init__(self) :
    self.frame_w = 640
    self.frame_h = 480

    self.roi_x = 80
    self.roi_y = 240
    self.roi_width = self.frame_w - self.roi_x
    self.roi_height = self.frame_h - self.roi_y

    self.black_h_low = 0
    self.black_h_high = 255
    self.black_s_low = 0
    self.black_s_high = 60
    self.black_v_low = 0
    self.black_v_high =100

    self.slope_treshold = 45

    rospy.init_node('line_det')
    self.bridge = CvBridge()

    self.mask_img = None
    self.det_img = None

    self.line_pub = rospy.Publisher('/line_det/line_img', Image, queue_size=1)
    self.mask_pub = rospy.Publisher('/line_det/mask_img', Image, queue_size=1)
    self.cam_sub = rospy.Subscriber('cam_pub/raw_img', Image, self.callback)

  def callback(self, msg) :
    selected_lines = []
    try :
      frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e :
      rospy.logerr(e)

    frame = frame[self.roi_y : , self.roi_x :]

    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    black_mask = cv2.inRange(img_hsv, (self.black_h_low, self.black_s_low, self.black_v_low), (self.black_h_high, self.black_s_high, self.black_v_high))

    img_black = cv2.bitwise_and(frame, frame, mask=black_mask)

    self.mask_img = img_black.copy()

    img_gray = cv2.cvtColor(img_black, cv2.COLOR_BGR2GRAY)
    img_edge = cv2.Canny(img_gray, 50, 450)

    lines = cv2.HoughLinesP(img_edge, 1, np.pi/180, 50, 20, 10)

    if lines is not None :
      selected_lines = []
      pts = []

      for line in lines :
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1 + 1e-16)
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2, cv2.LINE_AA)

        if abs(slope) >= np.tan( (90-self.slope_treshold) * np.pi / 180) :
          selected_lines.append(line)
          pts.append((x1, y1))
          pts.append((x2, y2))

      if len(pts) > 0 :
        pts = np.array(pts)
        fit_line = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        m = fit_line[1] / fit_line[0]
        b = (fit_line[2], fit_line[3])



        pt1_y = frame.shape[0]
        pt2_y = 0
        pt1_x = (((pt1_y - b[1]) / m) + b[0]).astype(np.int8)
        pt2_x = (((pt2_y - b[1]) / m) + b[0]).astype(np.int8)

        pt1_y = np.array([pt1_y]).astype(np.int8)
        pt2_y = np.array([pt2_y]).astype(np.int8)

        rospy.loginfo((pt1_x, pt1_y))
        rospy.loginfo((pt2_x, pt2_y))
        cv2.line(frame, (pt1_x[0], pt1_y[0]) , (pt2_x[0], pt2_y[0]), (0,0,255), 2, cv2.LINE_AA)

    for line in selected_lines :
      x1, y1, x2, y2 = line[0]
      cv2.line(frame, (x1, y1), (x2, y2), (255, 0,0), 2, cv2.LINE_AA)

    self.det_img = frame.copy()

    try :
      pub_det = self.bridge.cv2_to_imgmsg(self.det_img, encoding='bgr8')
      pub_line = self.bridge.cv2_to_imgmsg(self.mask_img, encoding='bgr8')

      self.line_pub.publish(pub_line)
      self.mask_pub.publish(pub_det)

    except CvBridgeError as e:
      rospy.logerr(e)


  def det(self) :
    rospy.spin()


if __name__ == '__main__' :
  node = LineDetectnode()
  node.det()


