import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if not cap.isOpened() :
  print('Camera is Not oppend !')

pallet_h_min = 40
pallet_h_max = 60

pallet_s_min = 17
pallet_s_max = 100

pallet_v_min = 30
pallet_v_max = 100

pallet_lower = (int(pallet_h_min/2), int(pallet_s_min*2.55), int(pallet_v_min*2.55))
pallet_upper = (int(pallet_h_max/2), int(pallet_s_max*2.55), int(pallet_v_max*2.55))

while True :
  ret, frame = cap.read()

  hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  h,s,v = cv2.split(hsv_img)

  pallet_mask = cv2.inRange(hsv_img, pallet_lower, pallet_upper)

  pallet_img = cv2.bitwise_and(frame, frame, mask=pallet_mask)

  cv2.imshow('h', h)
  cv2.imshow('s', s)
  cv2.imshow('v', v)
  cv2.imshow('test',pallet_img)
  cv2.waitKey(1)

