#!/usr/bin/env python

import numpy as np
from ultralytics import YOLO

class ObjectDetector:
    def __init__(self):
        self.model = YOLO('./yolov8n.pt')
        
    def __call__(self, cv_image):
        detected_objects = self.detect_objects(cv_image)
        return detected_objects

    def detect_objects(self, image):
        results = self.model(image)
        detected_objects = []
        
        for result in results[0].boxes:
            x1, y1, x2, y2 = result.xyxy[0].tolist()
            class_label = self.model.names[int(result.cls)]
            confidence = result.conf.item()
            detected_objects.append([x1, y1, x2, y2, confidence])
        
        return np.array(detected_objects, dtype=np.float32)