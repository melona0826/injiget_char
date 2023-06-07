from transformers import TrOCRProcessor, VisionEncoderDecoderModel
from PIL import Image
import cv2
import image_preprocess
import torch
from difflib import SequenceMatcher
import rospy
from sensor_msgs import Image

