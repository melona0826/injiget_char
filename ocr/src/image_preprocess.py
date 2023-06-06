from PIL import Image
import cv2
from imutils.perspective import four_point_transform
from imutils.contours import sort_contours
import imutils
import matplotlib.pyplot as plt
import numpy as np

def plt_imshow(title='image', img=None, figsize=(8 ,5)):
    plt.figure(figsize=figsize)
 
    if type(img) == list:
        if type(title) == list:
            titles = title
        else:
            titles = []
 
            for i in range(len(img)):
                titles.append(title)
 
        for i in range(len(img)):
            if len(img[i].shape) <= 2:
                rgbImg = cv2.cvtColor(img[i], cv2.COLOR_GRAY2RGB)
            else:
                rgbImg = cv2.cvtColor(img[i], cv2.COLOR_BGR2RGB)
 
            plt.subplot(1, len(img), i + 1), plt.imshow(rgbImg)
            plt.title(titles[i])
            plt.xticks([]), plt.yticks([])
 
        plt.show()
    else:
        if len(img.shape) < 3:
            rgbImg = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        else:
            rgbImg = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
 
        plt.imshow(rgbImg)
        plt.title(title)
        plt.xticks([]), plt.yticks([])
        plt.show()

def make_scan_image(image, ksize=(5,5), min_threshold=100, max_threshold=200, max_count=2):
  image_list_title = []
  image_list = []
 
  org_image = image.copy()
  ratio = org_image.shape[1] / float(image.shape[1])

  gray = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  blurred = cv2.GaussianBlur(gray, ksize, 0)
  extracted = cv2.inRange(blurred, (0,0,255-80), (255,80,255))
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
 
  if len(find_cnts) < max_count:
    raise Exception(("Could not find outline."))

  image_list_title.append("Outline")
  image_list.append(output)
  
#   for i in range(len(image_list)):
#     plt_imshow(image_list_title[i], image_list[i])
  
  for i,fc in enumerate(sorted(find_cnts, key=lambda x: np.min(x.reshape(4,2), axis=0)[0])):
      transform_image = four_point_transform(org_image, fc.reshape(4, 2) * ratio)
      transform_images.append(transform_image)
      find_places.append(np.mean(fc.reshape(4,2), axis=0).tolist())
    #   plt_imshow(i, transform_image)
  
  return transform_images, find_places

