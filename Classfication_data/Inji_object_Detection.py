from ultralytics import YOLO
import torch
import cv2 
import numpy as np
import pathlib
import matplotlib.pyplot as plt

img = cv2.imread("injigayinput.jpg")
model = YOLO("model_- 4 june 2023 17_04.pt")
results = model(img)
res_plotted = results[0].plot()

for result in results:
    boxes = result.boxes  # Boxes object for bbox outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    probs = result.probs  # Class probabilities for classification outputs
