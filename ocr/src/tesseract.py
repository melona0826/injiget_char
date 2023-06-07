from PIL import Image
import cv2
import image_preprocess
import torch
from difflib import SequenceMatcher
import pytesseract

def process_image(images):
    results = []
    for image in images:
      generated_text = pytesseract.image_to_string(image, lang='eng')
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

def tesseract(path, ground_truth, cnt, detail):
  image = cv2.imread(path)
 
  detected_images, detected_places = image_preprocess.make_scan_image(image, ksize=(5, 5), min_threshold=20, max_threshold=150, max_count=cnt, detail=detail)
  results = process_image(detected_images)
  answers = get_answer(ground_truth, results)
  print("=====Tesseract OCR Result=====")
  print(results)
  print("=====Tesseract OCR Corrected Result=====")
  print(answers)
  print(detected_places)
  
  return answers, detected_places

if __name__ == "__main__":
  tesseract('../assets/test10.jpg', ["welchs", "cantata", "oronamin", "tejava", "demisoda"], 4, detail=False)