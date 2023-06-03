import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if not cap.isOpened() :
  print('Camera is Not oppend !')

pallet_h_min = 22
pallet_h_max = 38

pallet_s_min = 90
pallet_s_max = 255

pallet_v_min = 200
pallet_v_max = 255

while True :
  ret, frame = cap.read()

  hsv_img = cv2.imshow


  cv2.imshow('test',frame)
  cv2.waitKey(1)


