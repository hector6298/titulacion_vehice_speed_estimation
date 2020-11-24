import numpy as np
import cv2
import matplotlib.pyplot as plt

from skimage import feature,color,transform,io



########################

def getEdgeMap(grayImg, min, max):
    grayImgCpy = grayImg.copy()
    return cv2.Canny(grayImgCpy,min,max)

def getLinesFromEdges(edgeMap, angleBounds: tuple, line_length=35, line_gap=1):
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
    imgCpy = img.copy()
    for p0, p1 in lines:
        imgCpy = cv2.line(imgCpy, p0,p1,color,thickness)
    
    return imgCpy

########

def convertImg2Grayscale(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)