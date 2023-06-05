import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

class DetectNode :
  def __init__ (self) :
    rospy.init_node('pallet_detect')
    self.pub = rospy.Publisher('/pallet_det/pallet_img', Image, queue_size=1)
    rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
    self.bridge = CvBridge()
    self.minHessian = 20
    self.ref_img = cv2.imread('refer.png')
    if self.ref_img is None :
      rospy.logerr("Reference Img is Not Loaded !")
    self.ref_img = cv2.cvtColor(self.ref_img, cv2.COLOR_BGR2GRAY)

  def callback(self, msg) :
    try :
      frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
      frame = cv2.flip(frame, -1)
      frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      detector = cv2.xfeatures2d.SURF_create(hessianThreshold=self.minHessian)

      key_obj, descrip_obj = detector.detectAndCompute(self.ref_img, None)
      key_scene, descrip_scene = detector.detectAndCompute(frame, None)

      matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
      knn_matches = matcher.knnMatch(descrip_obj, descrip_scene, 2)

      threshold = 0.75
      good_match = []
      for m,n in knn_matches :
        if m.distance < threshold * n.distance :
          good_match.append(m)

      img_matches = np.empty((max(self.ref_img.shape[0], frame.shape[0]), self.ref_img.shape[1] + frame.shape[1], 3), dtype=np.uint8)
      cv2.drawMatches(self.ref_img, key_obj, frame, key_scene, good_match, img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

      obj = np.empty((len(good_match), 2), dtype=np.float32)
      scene = np.empty((len(good_match), 2), dtype=np.float32)

      for i in range(len(good_match)) :
        obj[i,0] = key_obj[good_match[i].queryIdx].pt[0]
        obj[i,1] = key_obj[good_match[i].queryIdx].pt[1]
        scene[i,0] = key_scene[good_match[i].trainIdx].pt[0]
        scene[i,1] = key_scene[good_match[i].trainIdx].pt[1]



      M, mask = cv2.findHomography(obj,scene,cv2.RANSAC, 5.0)
      matchesMask = mask.ravel().tolist()

      h,w = frame.shape
      pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
      dst = cv2.perspectiveTransform(pts,M)

      img_matches = cv2.polylines(img_matches,[np.int32(dst)],True,255,3,cv2.LINE_AA)




      msg = self.bridge.cv2_to_imgmsg(img_matches, 'bgr8')
      self.pub.publish(msg)

    except Exception as e :
      rospy.logerr(e)




if __name__ == '__main__' :
  det_node = DetectNode()
  rospy.spin()
