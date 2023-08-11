import ultralytics
from ultralytics import YOLO
import cv2
import torch

print(torch.cuda.is_available)

ultralytics.checks()
