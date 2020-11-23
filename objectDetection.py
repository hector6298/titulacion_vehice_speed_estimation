import os
import datetime
import torch
import cv2
import numpy as np

from PIL import Image
from torch.autograd import Variable

from yolov3.models import *
from yolov3.utils.utils import *
from yolov3.utils.datasets import *

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class objectDetector(object):
    """
    Object Detector high level object using YOLOv3 from https://github.com/eriklindernoren/PyTorch-YOLOv3.git
    """
    def __init__(self, imgSize, weightPath, classPath, modelDef, confidenceThres=0.7, IoUThres=0.4):
        self.imgSize = imgSize
        self.confidenceThres = confidenceThres
        self.IoUThres = IoUThres
        self.model = Darknet(modelDef, img_size=imgSize).to(device)
        self.tensorType = torch.cuda.FloatTensor if torch.cuda.is_available()\
                                                 else torch.FloatTensor
        self.trf = T.Compose([
                        T.ToTensor(), 
                        T.Normalize(mean = [0.485, 0.456, 0.406], 
                                    std = [0.229, 0.224, 0.225])])
        if weightPath.endswith(".weights"):
            # Load darknet weights
            self.model.load_darknet_weights(weightPath)
        else:
            # Load checkpoint weights
            self.model.load_state_dict(torch.load(weightPath))
        self.model.eval()

    def detect(self, img):
        imgPIL = Image.fromarray(img)
        inp = self.trf(img).unsqueeze(0)
        with torch.no_grad():
            detections = model(inp)
            detections = non_max_suppression(detections, 
                                             self.confidenceThres, 
                                             self.IoUThres)
        if detections is not None:
            #x1, y1, x2, y2, conf, confidence, class pred
            detections = rescale_boxes(detections, opt.img_size, img.shape[:2])
        return detections

class objectVisualizer(object):
    def __init__(self, classPath, color = (0,0,255), thickness=1):
        self.color = color
        self.thickness = thickness
        self.classes = load_classes(classPath)
        self.textColor = textColor
        self.textThickness = textThickness

    def drawBoxes(self, img, detections):
        imgCpy = img.copy()
        if detections is not None:
            unique_labels = detections[:, -1].cpu().unique()
            n_cls_preds = len(unique_labels)
            for x1, y1, x2, y2, conf, cls_conf, cls_pred in detections:
                imgCpy = cv2.Rectangle(imgCpy,(x1,y1),(x2,y2))
                imgCpy = cv2.putText(imgCpy, f"{self.classes[cls_pred]} {cls_conf}",
                                     (x1,y1), cv2.FONT_HERSHEY_COMPLEX_SMALL,1, self.textColor, self.textThickness)
        return imgCpy