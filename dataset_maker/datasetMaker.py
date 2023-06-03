import cv2
import os
import sys

def makeDirs(path):
  try:
    if not os.path.exists(path):
        os.makedirs(path)
  except OSError:
    print('Error !')

cap = cv2.VideoCapture(0)

object_name = 'aa'
save_path = './datasets/'

save_path += object_name

makeDirs(save_path + object_name)

i = 0
print("=== Start " + str(object_name) + " Dataset Maker ===")
print("====== Type Any Characater for Start ======")
input()

print("--- Press ESC for Exit ---")

while True :
  ret , frame = cap.read()

  if ret :
    cv2.imshow('video',frame)
    cv2.imwrite(save_path + '/img' + str(i) + '.jpg' , frame)
    if cv2.waitKey(50) == 27 :
      break
    i += 1

  else :
    print('Camera Connect Failed !')

    break

cap.release()
cv2.destroyAllWindows()

