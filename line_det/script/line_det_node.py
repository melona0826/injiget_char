import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if not cap.isOpened() :
  print('Camera is Not oppend !')

line_hue

while True :
  ret, frame = cap.read()

  cv2.imshow('test',frame)
  cv2.waitKey(1)


