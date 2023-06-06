import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def stream() :
  cap = cv2.VideoCapture(0)
  rospy.init_node("cam_pub", anonymous=True)
  img_pub = rospy.Publisher("cam_img_pub", Image, queue_size=1)

  bridge = CvBridge()
  rate = rospy.Rate(1)
  while not rospy.is_shutdown() :
    ret, frame = cap.read()
    img_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    rate.sleep()

if __name__ == '__main__' :
  try :
    stream()
  except rospy.ROSInterruptException :
    pass
