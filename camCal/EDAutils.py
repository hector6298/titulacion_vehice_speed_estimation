import numpy as np
import cv2

#rotates a point by a given angle

def rotPt(oPt:np.ndarray, fAng:float) -> np.ndarray:
    return np.array([
        ((oPt[1]*np.cos(fAng)) - (oPt[0]* np.sin(fAng))), 
        ((oPt[1]*np.sin(fAng)) + (oPt[0]* np.cos(fAng)))
        ])

def get_rand_num(max:float, min:float, seed:int):
    rand = random.rand()
    duration = max - min
    return min + rand*duration

def proj3d22d(o3dPt:np.ndarray, afProjMat:np.ndarray, nLenUnit:int=1):
    oMatP = np.reshape(afProjMat, (3,4)).astype(np.float32)
    oMatM3d = np.empty((4,1), dtype=np.float32)
    oMatM2d = np.empty((3,1), dtype=np.float32)

    oMatM3d[0,0] = o3dPt[0] * nLenUnit   #x
    oMatM3d[1,0] = o3dPt[1] * nLenUnit   #y
    oMatM3d[2,0] = o3dPt[2] * nLenUnit   #z
    oMatM3d[3,0] = 1.0

    oMatM2d = np.matmul(oMatP, oMatM3d)

    o2dPt = ((oMatM2d[0,0]/oMatM2d[2,0]), (oMatM2d[1,0]/oMatM2d[2,0]))
    return o2dPt

def bkproj2d23d(o2dPt:np.ndarray,  afProjMat:np.ndarray, nLenUnit:int=1, nCoordSysTyp:int=1):
    oMatA = np.empty((3,3), dtype=np.float64)

    if nCoordSysTyp == 0:
        oMatA[0,0] = afProjMat[0]
        oMatA[0,1] = -o2dPt[1]
        oMatA[0,2] = afProjMat[2]
        oMatA[1,0] = afProjMat[4]
        oMatA[1,1] = -o2dPt[0]
        oMatA[1,2] = afProjMat[6]
        oMatA[2,0] = afProjMat[8]
        oMatA[2,1] = -1.0
        oMatA[2,2] = afProjMat[10]
    
    if nCoordSysTyp == 1:
        oMatA[0,0] = afProjMat[0]
        oMatA[0,1] = afProjMat[1]
        oMatA[0,2] = -o2dPt[1]
        oMatA[1,0] = afProjMat[4]
        oMatA[1,1] = afProjMat[5]
        oMatA[1,2] = -o2dPt[0]
        oMatA[2,0] = afProjMat[8]
        oMatA[2,1] = afProjMat[9]
        oMatA[2,2] = -1.0

    oMatAInv = np.linalg.inv(oMatA)
    oMatB = np.empty((3,1), dtype=np.float64)
    oMatB[0,0] = -afProjMat[3]
    oMatB[1,0] = -afProjMat[7]
    oMatB[2,0] = -afProjMat[11]

    oMatM = np.matmul(oMatAInv, oMatB)

    o3dPt = None

    if nCoordSysTyp == 0:
        o3dPt = (oMatM[0,0], 0.0, oMatM[2,0])/nLenUnit
    elif nCoordSysTyp == 1:
        o3dPt = (oMatM[0,0], oMatM[1,0], 0.0)/nLenUnit

    return o3dPt
