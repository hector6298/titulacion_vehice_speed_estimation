import cv2
from scipy.spatial import distance
import numpy as np
from collections import OrderedDict
from img2real import reproject2Dto3D, parseParams
import math

class Tracker:
    def __init__(self, height, width, maxlost = 30, speed_update_rate = 10, params_file=None):
        self.speed_update_rate = speed_update_rate
        self.nextObjId = 0
        self.objs = OrderedDict()
        #### KEEP EYE ON THIS #####
        self.objsNewestRealLoc = OrderedDict()
        self.objsPrevRealLoc = OrderedDict()
        self.frameCount = OrderedDict()
        self.objsRealVel = OrderedDict()
        ################################
        self.lost = OrderedDict()
        self.maxLost = maxlost
        self.height = height
        self.width = width
        self.focal_length = 663.429
        self.projectionMat = [[ 8.24264e-01, -3.95238e-01, -4.05434e-01, -1.98855e+03],
                             [ 2.13082e-01,  8.79944e-01, -4.24611e-01,  2.78921e+03],
                             [ 5.24581e-01,  2.63600e-01,  8.09524e-01,  6.93513e+03]]             
        self.projectionMat = np.array(self.projectionMat)

    def addObject(self, new_object_location):
        self.objs[self.nextObjId] = new_object_location
        self.objsNewestRealLoc[self.nextObjId] = None
        self.objsPrevRealLoc[self.nextObjId] = reproject2Dto3D(new_object_location, self.focal_length, self.projectionMat)
        self.objsRealVel[self.nextObjId] = 0.0
        self.frameCount[self.nextObjId] = 10
        self.lost[self.nextObjId] = 0
        self.nextObjId += 1

    def removeObject(self, objectID):
        del self.objs[objectID]
        del self.objsNewestRealLoc[objectID]
        del self.lost[objectID]
        del self.objsRealVel[objectID]
        del self.objsPrevRealLoc[objectID]
        del self.frameCount[objectID]

    
    def getLocation(self,bbox):
        x1, y1, x2, y2 = bbox
        return (int((x1 + x2) /2.0),int((y1 + y2 )/2.0))

    def update(self, bboxes, frameRate):
        if len(bboxes) == 0:
            lost_ids = list(self.lost.keys())
            for objectID in lost_ids:
                self.lost[objectID] += 1
                if self.lost[objectID] > self.maxLost: self.removeObject(objectID)
            return self.objs, self.objsRealVel
        
        new_object_locations = np.zeros((len(bboxes), 2), dtype="int")
        for i, bbox in enumerate(bboxes):
            new_object_locations[i] = self.getLocation(bbox)
        
        if len(self.objs) == 0:
            for i in range(len(bboxes)):
                self.addObject(new_object_locations[i])
        else:
            objectIDs = list(self.objs.keys())
            previous_object_locations = np.array(list(self.objs.values()))
            

            D = distance.cdist(previous_object_locations, new_object_locations)

            row_idx = D.argmin(axis=1).argsort()
            cols_idx = D.argmin(axis=1)[row_idx]

            assignedRows, assignedCols = set(), set()
            for row, col in zip(row_idx, cols_idx):
                if row in assignedRows or col in assignedCols:
                    continue
                objectID = objectIDs[row]
                self.objs[objectID] = new_object_locations[col]
                self.objsNewestRealLoc[objectID] = reproject2Dto3D(self.objs[objectID], self.focal_length, self.projectionMat)
                self.frameCount[objectID] -= 1
                if(self.frameCount[objectID]  == 0):
                    #componentDist = (self.objsNewestRealLoc[objectID] - self.objsPrevRealLoc[objectID])
                    componentDist = np.subtract(self.objsNewestRealLoc[objectID], self.objsPrevRealLoc[objectID])
                    euclideanDist = math.sqrt(componentDist[0]**2 + componentDist[1]**2)
                    self.objsRealVel[objectID] = (euclideanDist/((self.lost[objectID]+self.speed_update_rate)/frameRate))*(3600/1000000)
                    self.objsPrevRealLoc[objectID] = self.objsNewestRealLoc[objectID]
                    self.frameCount[objectID] = self.speed_update_rate
                self.lost[objectID] = 0

                assignedRows.add(row)
                assignedCols.add(col)
            unassignedRows = set(range(0, D.shape[0])).difference(assignedRows)
            unassignedCols = set(range(0, D.shape[1])).difference(assignedCols)

            if D.shape[0] >= D.shape[1]:
                for row in unassignedRows:
                    objectID = objectIDs[row]
                    self.lost[objectID] += 1
                    if self.lost[objectID] > self.maxLost:
                        self.removeObject(objectID)
            else:
                for col in unassignedCols:
                    self.addObject(new_object_locations[col])
        return self.objs, self.objsRealVel

#### ADD FOR REAL WORLD DISTANCEobjs