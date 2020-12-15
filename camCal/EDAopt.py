import numpy as np
import cv2

from EDAutils import *
from camCalObjs import *

#constants
EDA_INIT_POP = 20000
EDA_SEL_POP = 20
EDA_ITER_NUM = 100

vanishingPoints = get2VanishingPoints()
m_oVr = vanishingPoints[0]
m_oVl = vanishingPoints[1]

class CCamCal(Object):
    def __init__(self, oCfg, oImgBg ):

        #configuration params
        self.m_oCfg = oCfg
        #background image for plotting results
        self.m_oImgBg = oImgBg
        self.m_bCol25ProgFlg = False
        self.m_bCol50ProgFlg = False
        self.m_bCol75ProgFlg = False
        self.m_voVyCand = None
        self.m_voLinfCand = None
        self.m_oVy = None
        self.m_fLinfSlp = none
        self.m_fLinfItcp = None
        self.m_fLinfCorr = None
        self.m_oVr = None
        self.m_oVl = None
        self.m_oPrinPt = None
    
    def process(self, voVanPt):
        if len(voVanPt) == 2:
            m_oVr = voVanPt[0]
            m_oVl = voVanPt[1]
            self.estPrinPtByAc()

            if self.m_oCfg.m_bCalEdaOptFlg == 0:
                self.calCamDctComp()
            elif self.m_oCfg.m_bCalEdaOptFlg == 1:
                self.calCamEdaOpt()
            
            return True
        else:
            print("Two vanishing points are needed in order to estimate camera parameters...")
            return False

    def estPrinPtByAc(self):
        self.m_oPrinPt[1] = self.m_oCfg.m_oFrmSz[1] / 2.0
        self.m_oPrinPt[0] = self.m_oCfg.m_oFrmSz[0] / 2.0

#class CCamCal
    def compCamParam(self, OVr, oVl, oPrincipalPt, fCamHeight):
        oVrC = np.empty((2,))
        oVlC = np.empty((2,))
        oVrCRotation = np.empty((2,))
        OVlCRotation = np.empty((2,))

        oVrC[1] = oVr[1] - oPrincipalPt[1]   #x
        oVrC[0] = oPrincipalPt[0] - oVr[0]    #y
        oVlC[1] = oVl[1] - oPrincipalPt[1]   #x
        oVlC[0] = oPrincipalPt[0] - oVl[0]    #y

        fRoll = np.rctan2((oVrC[0] - oVlC[0]), (oVrC[1] - oVlC[1]))
        fRoll = fRoll - np.pi if fRoll > (np.pi/2.0) else fRoll
        fRoll = fRolll + np.pi if fRoll < (-np.pi/2.0) else fRoll

        oVrCRotation = rotPt(oVrC, -fRoll)
        oVlCRotation = rotPt(oVlC, -fRoll)

        #why the - sign?
        fF = np.sqrt(-((oVrCRotation[0]*oVrCRotation[0]) + (oVrCRotation[1]*oVrCRotation[1])))

        if COORD_SYS_TYP == 0:
            fPitch = -np.arctan2(oVrCRotation[0], fF)
            fYaw = -np.arctan2(fF, (oVrCRotation[1]*np.cos(fPitch)))
        elif COORD_SYS_TYP == 1:
            fPitch = -np.arctan2(oVrCRotation[0], fF)
            fYaw = -np.arctan2((oVrCRotation[1]*np.cos(fPitch)), fF)

        camParam = CCamParam()

        camParam.m_fFx = fF
        camParam.m_fFy = fF
        camParam.m_fCx = oPrincipalPt[1] #x
        camParam.m_fCy = oPrincipalPt[0] #y
        camParam.m_fRoll = fRoll
        camParam.m_fPitch = fPitch
        camParam.m_fYaw = fYaw
        camParam.m_fTx = 0.0

        if COORD_SYS_TYP == 0:
            camParam.m_fTy = -fCamHeight*self.m_oCfg.m_nLenUnit
            camParam.mfTz = 0.0
        if COORD_SYS_TYP == 1:
            camParam.m_fTy = 0.0
            camParam.m_fTz = fCamHeight*self.m_oCfg.m_nLenUnit
    
        camParam.setInParamMat(fF, fF, oPrincipalPt[1], oPrincipalPt[0])
        camParam.setRotMat(fRoll, fPitch, fYaw)

        if COORD_SYS_TYP == 0:
            camParam.setTntMat(0.0, (-fCamHeight*self.m_oCfg.m_nLenUnit), 0.0)
        elif COORD_SYS_TYP == 1:
            camParam.setTntMat(0.0,0.0, (fCamHeight * self.m_oCfg.m_nLenUnit)
        
        camParam.calcProjMat()

        return camParam

    def tstStGrdPt(self, oStGrdPt:tuple, camParam:CCamParam):
        nLftEdgX = (-(IMG_EXPN_RAT - 1.0) / 2.0) * self.m_oCfg.m_oFrmSz[1]
        nLftEdgX = (-(IMG_EXPN_RAT - 1.0) / 2.0) * self.m_oCfg.m_oFrmSz[0]
        nLftEdgX = ((IMG_EXPN_RAT + 1.0) / 2.0) * self.m_oCfg.m_oFrmSz[1]
        nLftEdgX = ((IMG_EXPN_RAT + 1.0) / 2.0) * self.m_oCfg.m_oFrmSz[0]
        
        o2dStPt = (-1.0, -1.0)
        o2dRNdPt = (-1.0,-1.0)
        o2dLNdPt = (-1.0,-1.0)
        o2dNdPt = (-1.0, -1.0)

        oNdGrdPt = np.array((2,))
        oNdGrdPt[1] = oNdGrdPt[1] + self.m_oCfg.m_nCalGrdSzR
        oNdGrdPt[0] = oNdGrdPt[0] + self.m_oCfg.m_nCalGrdSzL

        if COORD_SYS_TYP == 0:
            o2dStPt = proj3d22d((oStGrdPt[1], 0.0, oStGrdPt[0]), camParam.m_afP, self.m_oCfg.m_nLenUnit)
            o2dRNdPt = proj3d22d((oNdGrdPt[1], 0.0, oStGrdPt[0]), poCamParam.m_afP, self.m_oCfg.m_nLenUnit)
            o2dLNdPt = proj3d22d((oStGrdPt[1], 0.0, oNdGrdPt[0]), poCamParam.m_afP, self.m_oCfg.m_nLenUnit)
            o2dNdPt = proj3d22d((oNdGrdPt[1], 0.0, oNdGrdPt[0]), poCamParam.m_afP, self.m_oCfg.m_nLenUnit)
        elif COORD_SYS_TYP == 1:
            o2dStPt = proj3d22d((oStGrdPt[1], oStGrdPt[0], 0.0), camParam.m_afP, self.m_oCfg.m_nLenUnit)
            o2dRNdPt = proj3d22d((oNdGrdPt[1], oStGrdPt[0], 0.0), poCamParam.m_afP, self.m_oCfg.m_nLenUnit)
            o2dLNdPt = proj3d22d((oStGrdPt[1], oNdGrdPt[0], 0.0), poCamParam.m_afP, self.m_oCfg.m_nLenUnit)
            o2dNdPt = proj3d22d((oNdGrdPt[1], oNdGrdPt[0], 0.0), poCamParam.m_afP, self.m_oCfg.m_nLenUnit)
    
    def calcStGrdPt(self, camParam):
        bStGrdPtFlg = False
        nMaxSumSqDist = 0
        voPt = []
        oStGrdPt = np.empty((2,))
        while True:
            nMaxDist = np.sqrt(nMaxSumSqDist)
            
            for i in range(nMaxDist+1):
                for j in range(i,nMaxDist+1):
                    if nMaxSumSqDist == (i*i)+(j*j):
                        voPt.append((i,j))
            
            if len(voPt):
                for i in range(len(voPt)):
                    oStGrdPt[1] = voPt[i][1]
                    oStGrdPt[0] = voPt[i][0]

    
    def calCamEdaOpt(self):

        nr = EDA_INIT_POP
        nN = EDA_SEL_POP
        NIterNum = EDA_ITER_NUM
        iIter = 0

        #set Starting Grid Point
        camParam = compCamParam(self.m_oVr, self.m_oVl, self.m_oPrinPt, ((self.m_oCfg.m_fCalCamHeiMax + self.m_oCfg.m_fCalCamHeiMin)/ 2.0))
        oStGrdPt = calcStGrdPt(camParam)