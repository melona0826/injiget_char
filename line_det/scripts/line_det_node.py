import cv2
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



frame_w = 640
frame_h = 480

roi_x = 80
roi_y = 240
roi_w = frame_w - roi_x
roi_h = frame_h - roi_y

black_h_low = 0
black_h_up = 255

black_s_low = 0
black_s_up = 255

black_v_low = 0
black_v_up = 111

black_hsv_low = (black_h_low, black_s_low, black_v_low)
black_hsv_up = (black_h_up, black_s_up, black_v_up)

class DetectNode(object) :
  def __init__(self) :
    self.img = None
    self.det_img = None
    self.line_img = None

    self.bridge = CvBridge()

    self.loop_rate = rospy.Rate(10)

    self.det_img_pub = rospy.Publisher("/line_det/det_img", Image, queue_size=1)
    self.line_img_pub = rospy.Publisher("/line_det/line_img", Image, queue_size=1)

    rospy.Subscriber("/cam_pub/raw_img", Image, self.callback())

  def callback(self, raw_img_msg) :
    self.frame = self.bridge.imgmsg_to_cv2(raw_img_msg)

    frame = img[y:y+roi_h, x:x+roi_w]

    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    black_mask = cv2.inRange(hsv_img, black_hsv_low, black_hsv_up)

    line_img = cv2.bitwise_and(frame, frame, mask=balck_mask)

    raw_img = frame.copy()

    line_img = cv2.cvtColor(line_img, cv2.COLOR_BGR2GRAY)
    edge_img = cv2.Canny(line_img, 50, 450)

    lines = HoughLinesP(edge_img, 1, cv2.CV_PI/180, 50, 20, 10)

    rospy.loginfo(type(lines))

  def detect(self):
    self.loop_rate.sleep()


if __name__ == '__main__' :
  rospy.init_node("line_det", anonymous=True)
  detNode = DetectNode()
  detNode.detect()


