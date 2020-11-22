import numpy as np
import cv2
import matplotlib.pyplot as plt

from skimage import feature,color,transform,io

def getBoundingBoxEnds(outputs):
    boxes, nums = (outputs[0][0], outputs[3][0])
    wh = np.flip(img.shape[0:2])
    boxpoints = []

    for i in range(nums):
        x1y1 = tuple((np.array(boxes[i][0:2])*wh)\
                .astype(np.int32))
        x2y2 = tuple((np.array(boxes[i][2:4]) * wh)\
                .astype(np.int32))
        boxpoints.append((x1y1[0],x1y1[1],x2y2[0],x2y2[1]))

    return boxpoints

def drawBoundingBoxes(img, boxpoints):
    imgCpy = img.copy()
    for points in boxpoints:
        imgCpy = cv2.rectangle(imgCpy,
                                (points[0],points[1]),
                                (points[2],points[3]))
    return imgCpy


########################

def getLinesFromEdges(edgeMap, draw, angleBounds: tuple, line_length=35, line_gap=1):
    lines = transform.probabilistic_hough_line(edgeMap, line_length=line_length, line_gap=line_gap)
    filteredLines = []
    for p0, p1 in lines:
        delta = np.array(p1) - np.array(p0)
        theta = np.arctan(float(delta[1])/delta[0])

        if theta > (np.pi/180)*angleBounds[0]\
           and theta < (np.pi/180)*angleBounds[1]:
            filteredLines.append((p0,p1))
    return filteredLines

def drawLinesOnImg(img, lines: list, color=(255,0,255), thickness=2):
    impCpy = img.copy()
    for p0, p1 in lines:
        imgCpy = cv2.line(imgCpy, p0,p1,color,thickness)
    
    return imgCpy

########

def convertImg2Grayscale(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)