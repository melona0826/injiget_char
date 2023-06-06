from transformers import TrOCRProcessor, VisionEncoderDecoderModel
from PIL import Image
import cv2
import image_preprocess
import torch
from difflib import SequenceMatcher

def process_image(images):
    results = []
    for image in images:
      pixel_values = processor(image, return_tensors="pt").to(device).pixel_values
      generated_ids = model.to(device).generate(pixel_values, max_length = 20)
      generated_text = processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
      results.append(generated_text)

    return results

def similar(a, b):
    return SequenceMatcher(None, a, b).ratio()

def get_answer(ground_truths, results):
    answers = []
    for word in results:
      similars = [similar(g, word.lower().strip()) for g in ground_truths]
      max_index = similars.index(max(similars))
      answers.append(ground_truths[max_index])
    
    return answers

def trocr(path, ground_truth, cnt):
  image = cv2.imread(path)
 
  detected_images, detected_places = image_preprocess.make_scan_image(image, ksize=(5, 5), min_threshold=20, max_threshold=150, max_count=cnt)
  results = process_image(detected_images)
  answers = get_answer(ground_truth, results)
  print("=====OCR Result=====")
  print(results)
  print("=====OCR Corrected Result=====")
  print(answers)
  print(detected_places)
  
  return answers, detected_places

if __name__ == "__main__":
  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  print(device)
  
  processor = TrOCRProcessor.from_pretrained("../models/trocr-base-handwritten")
  model = VisionEncoderDecoderModel.from_pretrained("../models/trocr-base-handwritten")
  
  # processor = TrOCRProcessor.from_pretrained("microsoft/trocr-base-handwritten")
  # model = VisionEncoderDecoderModel.from_pretrained("microsoft/trocr-base-handwritten")

  # processor.save_pretrained("../models/trocr-base-handwritten")
  # model.save_pretrained("../models/trocr-base-handwritten")
  
  trocr('../assets/test10.jpg', ["welchs", "cantata", "oronamin", "tejava", "demisoda"], 4)