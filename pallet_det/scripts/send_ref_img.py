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
    self.ref_img = cv2.imread('/home/jetson/catkin_ws/src/injiget_char/pallet_det/scripts/ref3.png')
    if self.ref_img is None :
      rospy.logerr("Reference Img is Not Loaded !")
    self.ref_img = cv2.cvtColor(self.ref_img, cv2.COLOR_BGR2GRAY)

    self.K = np.array([[539.33104, 0. , 315.999079], [0. , 537.486263, 203.796196], [0. , 0. , 1.0]])
    self.K_inv = np.linalg.inv(self.K)

  def callback(self, msg) :
    try :
      frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
      frame = cv2.flip(frame, -1)
      frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      detector = cv2.KAZE_create()
      #detector = cv2.xfeatures2d.SURF_create()

      kp1, des1 = detector.detectAndCompute(self.ref_img,None)
      kp2, des2 = detector.detectAndCompute(frame,None)

      FLANN_INDEX_KDTREE = 0
      index_params = dict(algorithm=FLANN_INDEX_KDTREE,trees=5)
      search_params = dict(checks=50)

      flann = cv2.FlannBasedMatcher(index_params,search_params)
      #matcher = cv2.BFMatcher_create()
      matches = flann.knnMatch(des1,des2,k=2)
      #matches = matcher.match(des1, des2)

      # matches = sorted(matches, key=lambda x: x.distance)
      # good = matches[:80]

      good = []
      for m,n in matches:
          if m.distance < 1.0*n.distance:
              good.append(m)

      MIN_MATCH_COUNT = 3

      if len(good) > MIN_MATCH_COUNT:
          src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
          dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1,1,2)

          H, mask = cv2.findHomography(src_pts,dst_pts,cv2.RANSAC, 3.0)
          norm = np.linalg.norm(H)

          m = np.matmul(self.K_inv, H / norm)

          c1 = m[:,0]
          c2 = m[:,1]
          c3 = np.cross(c1, c2)

          tvec = m[:,2]
          R = np.zeros((3,3))

          for i in range(3) :
            R[i][0] = c1[i]
            R[i][1] = c2[i]
            R[i][2] = c3[i]

          # rospy.loginfo(R)

          alpha = np.arctan2(R[1][0] , R[0][0]) * 180 / np.pi
          rospy.loginfo('A : ' + str(alpha))

          beta = np.arctan2(-R[2,0] , np.sqrt(R[2,1]**2 + R[2,2]**2))

          rospy.loginfo('B : ' + str(beta))

          gamma = np.arctan2(R[2][1] , R[2][2]) * 180 / np.pi
          rospy.loginfo('G : ' + str(gamma))

          rospy.loginfo('tvec : ' + str(tvec))
          #theta = np.arccos( (np.trace(R)-1) / 2 )
          #theta = -1 * 1 / ((1-((np.trace(R)-1)/2)**2) + 1e-24)
          # alpha = np.arctan2((-R[1,2]/ (R[2,2] + 1e-23)))
          # beta = np.arctan2((R[0,2] / (np.sqrt(1 - R[0,2]**2) + 1e-24)))
          #rospy.loginfo(alpha)
          #rospy.loginfo((-R[1,2]/ (R[2,2] + 1e-23)))
          #rospy.loginfo(beta)

          matchesMask = mask.ravel().tolist()

          h,w = self.ref_img.shape
          pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
          dst = cv2.perspectiveTransform(pts,H)

          frame = cv2.polylines(frame,[np.int32(dst)],True,255,3,cv2.LINE_AA)

      else:
          rospy.logerr('Not Enough Matching Point !')
          matchesMask = None

      draw_params = dict(matchColor = (0,255,0),
                  singlePointColor = None,
                  matchesMask=matchesMask,
                  flags=2)
      res=None
      res = cv2.drawMatches(self.ref_img,kp1,frame,kp2,good,res,**draw_params)

      msg = self.bridge.cv2_to_imgmsg(res, 'bgr8')
      self.pub.publish(msg)

    except Exception as e :
      rospy.logerr(e)




if __name__ == '__main__' :
  det_node = DetectNode()
  rospy.spin()
