from ultralytics import YOLO
import torch
import cv2
import numpy as np
import pathlib
import matplotlib.pyplot as plt


img = [cv2.imread("aaa.png")]
model = YOLO("model_- 4 june 2023 17_04.pt")
results = model(img)

print(int(results[0].boxes.cls[0].item())) # Label
print(results[0].boxes.conf[0].item())
