import numpy as np
import cv2

class foregroundExtractor(object):
    def __init__(self,
                kernelSize=(3,3)):
        self.subtractor = cv2.createBackgroundSubtractorKNN()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,kernelSize)

    def _getCarsMaskAtFrame(self,frame):
        assert len(frame.shape) == 2, f"frame must be on grayscale, found {len(frame.shape)} dimensions"
    
        foregroundMask = self.subtractor.apply(frame)
        foregroundMask[foregroundMask != 0] = 255
        foregroundMask = cv2.morphologyEx(foregroundMask, 
                                        cv2.MORPH_OPEN, 
                                        self.kernel)
        self.foregroundMask = cv2.morphologyEx(foregroundMask, cv2.MORPH_CLOSE, (7,7),iterations=10)

    def maskCarsAtFrame(self,frame):
        self._getCarsMaskAtFrame(frame)
        maskedFrame = cv2.bitwise_and(frame,frame,mask=self.foregroundMask)
        return maskedFrame