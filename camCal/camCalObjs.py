import numpy as np
import cv2
from numpy import random


class SParamRng(Object):
    def __init__(self):
        self.fFxMax = 5000
        self.fFxMin = 0

        self.fFyMax = 5000
        self.fFymin = 0

        self.fCxMax = 5000
        self.fCxMin = 0

        self.fCyMax = 5000
        self.fCymin = 0

        self.TxMax = 10
        self.fTxMin = -10

        self.fTyMax = 10
        self.fTyMin = -10

        self.fTzMax = 10
        self.fTzMin = -10

        self.fRollMax = deg2rad(90)
        self.fRollMin = deg2rad(-90)

        self.fPitchMax = deg2rad(90)
        self.fPitchMin = deg2rad(-90)

        self.fYawMax = deg2rad(90)
        self.fYawMin = deg2rad(-90)


class CCamParam(object):
    def __init__(self):
        self.m_afK = np.empty((9,), np.float32)
        self.m_afR  = np.empty((9,), np.float32)
        self.m_afT = np.empty((9,), np.float32)
        self.m_afP = np.empty((9,), np.float32)

        self.m_fFx = 0.0
        self.m_fFy = 0.0
        self.m_fCx = 0.0
        self.m_fCy = 0.0
        self.m_fRoll = 0.0
        self.m_fPitch = 0.0
        self.m_fYaw = 0.0
        self.m_fTx = 0.0
        self.m_fTy = 0.0
        self.m_fTz = 0.0

        self.m_fReprojErr = 0.0


        def setInParamMat(fFx:float, fFy:float, fCx:float, fCy:float):
            self.m_afK[0] = fFx
            self.m_afK[1] = 0.0
            self.m_afK[0] = fCx
            self.m_afK[1] = 0.0
            self.m_afK[0] = fFy
            self.m_afK[1] = fCy
            self.m_afK[0] = 0.0
            self.m_afK[1] = 0.0
            self.m_afK[1] = 1.0
        
        def setRotMat(fRoll:float, fPitch:float, fYaw:float):
            if COORD_SYS_TYP == 0:
                self.m_afR[0] = (np.cos(fRoll)*np.cos(fYaw)) - (np.sin(fRoll)*np.sin(fPitch)*np.sin(fYaw))
                self.m_afR[1] = -np.sin(fRoll)*np.cos(fPitch)
                self.m_afR[2] = (np.cos(fRoll)*np.sin(fYaw)) + (np.sin(fRoll)*np.sin(fPitch)*np.cos(fYaw))
                self.m_afR[3] = (np.sin(fRoll)*np.cos(fYaw)) + (np.cos(fRoll)*np.sin(fPitch)*np.sin(fYaw))
                self.m_afR[4] = np.cos(fRoll)*np.cos(fPitch)
                self.m_afR[5] = (np.sin(fRoll)*np.sin(fYaw)) - (np.cos(fRoll)*np.sin(fPitch)*np.cos(fyaw))
                self.m_afR[6] = -np.cos(fPitch)*np.sin(fYaw)
                self.m_afR[7] = np.sin(fPitch)
                self.m_afR[8] = np.cos(fPitch)*np.cos(fYaw)

            elif COORD_SYS_TYP == 1:
                self.m_afR[0] = (-np.cos(fRoll)*np.sin(fYaw)) - (np.sin(fRoll)*np.sin(fPitch)*np.cos(fYaw))
                self.m_afR[1] = (-np.cos(fRoll)*np.cos(fYaw)) - (np.sin(fRoll)*np.sin(fPitch)*np.cos(fYaw))
                self.m_afR[2] = np.sin(fRoll)*np.cos(fPitch)
                self.m_afR[3] = (-np.sin(fRoll) * np.sin(fYaw)) + (np.cos(fRoll) * np.sin(fPitch) * np.cos(fYaw))
                self.m_afR[4] = -np.sin(fRoll) * np.cos(fYaw)) - (np.cos(fRoll) * np.sin(fPitch) * np.sin(fYaw))
                self.m_afR[5] = -np.cos(fRoll) * np.cos(fPitch)
                self.m_afR[6] = np.cos(fPitch) * np.cos(fYaw)
                self.m_afR[7] = -np.cos(fPitch) * np.sin(fYaw)
                self.m_afR[8] = np.sin(fPitch)
        
        def setTntMat(fTx:float, fTy:float, fTz:float):
            self.m_afT[0] = fTx
            self.m_afT[1] = fTy
            self.m_afT[2] = fTz

        def calcProjMat(self):
            oMatK = np.reshape(self.m_afK, (3,3))
            oMatR = np.reshape(self.m_afR, (3,3))
            oMatT = np.empty((3,1), np.float32)
            oMatP = np.empty((3,4), np.float32)

            oMatT[0,0] = -self.m_afT[0]
            oMatT[1,0] = -self.m_aft[1]
            oMatT[2,0] = -self.m_afT[2]
            oMatT = np.matmul(oMatR, oMatT)

            oMatP[0,0] = self.m_afR[0]
            oMatP[0,1] = self.m_afR[1]
            oMatP[0,2] = self.m_afR[2]
            oMatP[1,0] = self.m_afR[3]
            omatP[1,1] = self.m_afR[4]
            oMatP[1,2] = self.m_afR[5]
            oMatP[2,0] = self.m_afR[6]
            oMatP[2,1] = self.m_afR[7]
            oMatP[2,2] = self.m_afR[8]
            oMatP[0,3] = oMatT[0,0]
            oMatP[1,3] = oMatT[1,0]
            oMatP[2,3] = oMatT[2,0]
            oMatP = np.matmul(oMatK, oMatP)

            self.m_afP[0] = oMatP[0,0]
            self.m_afp[1] = oMatP[0,1]
            self.m_afP[2] = oMatP[0,2]
            self.m_afP[3] = omatP[0,3]
            self.m_afP[4] = oMatP[1,0]
            self.m_afp[5] = oMatP[1,1]
            self.m_afP[6] = oMatP[1,2]
            self.m_afP[7] = omatP[1,3]
            self.m_afP[8] = oMatP[2,0]
            self.m_afp[9] = oMatP[2,1]
            self.m_afP[10] = oMatP[2,2]
            self.m_afP[11] = omatP[2,3]

        def readCamInParamTxt(CamInParamPath:str):
            pass

        def initCamMdl(sparamRng:SParamRng):
            self.m_fFx = get_rand_num(sparamRng.fFxMin, sparamRng.fFxMax)
            self.m_fFy = get_rand_num(sparamRng.fFyMin, sparamRng.fFyMax)
            self.m_fCx = get_rand_num(sparamRng.fCxMin, sparamRng.fCxMax)
            self.m_fCy = get_rand_num(sparamRng.fCyMin, sparamRng.fCyMax)
            self.m_fRoll = get_rand_num(sparamRng.fRollMin, sparamRng.fRollMax)
            self.m_fPitch = get_rand_num(sparamRng.fPitchMin, sparamRng.fPitchMax)
            self.m_fYaw = get_rand_num(sparamRng.fYawMin, sparamRng.fYawMax)
            self.m_fTx = get_rand_num(sparamRng.fTxMin, sparamRng.fTxMax)
            self.m_fTy = get_rand_num(sparamRng.fTyMin, sparamRng.fTyMax)
            self.mfTz = get_rand_num(sparamRng.fTzMin, sparamRng.fTzMax)


class CCfg(Object):
    def __init__(self, lenUnit=1000):
        self.m_oFrmSz = None
        self.m_nRszFrmHei = None
        self.m_nLenUnit = lenUnit
        self.m_oCalVr = None
        self.m_oCalVl = None
        self.m_fCalCamHeiMax = None
        self.m_fCalCamHeiMin = None
        self.m_nCalGrdSzR = None
        self.m_nCalGrdSzL = None
        self.m_bCalEdaOptFlg = None
        self.m_OutCamParamPth = ""
        #pairs of points of measuring line segments
        self.m_voCalMeasLnSegNdPt = []
        #ground truth ditances of measuring line segments
        self.m_vfCalMeasLnSegDist = []

