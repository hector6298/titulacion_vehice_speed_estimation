import numpy as np
import cv2
import matplotlib.pyplot as plt
import torch
import torchvision.transforms as T

from PIL import Image
from matplotlib import cm

from PIL import Image
from torchvision import models

#class macros
BUS = 6
CAR = 7

class TraditionalForegroundExtractor(object):
    def __init__(self,
                kernelSize=(3,3)):
        self.subtractor = cv2.createBackgroundSubtractorMOG2()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,kernelSize)

    def _getCarsMaskAtFrame(self,frame):
        assert len(frame.shape) == 2, f"frame must be on grayscale, found {len(frame.shape)} dimensions"
    
        foregroundMask = self.subtractor.apply(frame)
        foregroundMask[foregroundMask != 0] = 255
        foregroundMask = cv2.morphologyEx(foregroundMask, 
                                        cv2.MORPH_OPEN, 
                                        self.kernel)
        self.foregroundMask = cv2.morphologyEx(foregroundMask, cv2.MORPH_CLOSE, (7,7),iterations=10)
        return self.foregroundMask 

    def maskCarsAtFrame(self,frame):
        self._getCarsMaskAtFrame(frame)
        maskedFrame = cv2.bitwise_and(frame,frame,mask=self.foregroundMask)
        return maskedFrame



class DLForegroundExtractor(object):
    def __init__(self, imgSize):
        self.dlab = models.segmentation.deeplabv3_resnet101(pretrained=1).eval()
        self.imgSize = imgSize

    def _getCarsMaskAtFrame(self,img):
        trf = T.Compose([
                        T.ToTensor(), 
                        T.Normalize(mean = [0.485, 0.456, 0.406], 
                                    std = [0.229, 0.224, 0.225])])
        inp = trf(img).unsqueeze(0)
        out = self.dlab(inp)['out']
        print("done")
        om = torch.argmax(out.squeeze(), dim=0).detach().cpu().numpy()
        om[om != 0] = 255
        return om

    def segment(self, img, gray):
        imgPIL = Image.fromarray(img)
        om = self._getCarsMaskAtFrame(imgPIL)
        maskedFrame = cv2.bitwise_and(gray,gray,mask=np.uint8(om))
        return maskedFrame
    
    #plt.imshow(rgb); plt.axis('off'); plt.show()
