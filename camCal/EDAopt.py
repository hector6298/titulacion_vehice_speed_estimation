import numpy as np
import cv2

from EDAutils import *
from camCalObjs import *
from EDAConstants import *


class CCamCal(Object):
    def __init__(self, oCfg, oImgBg ):

        #configuration params
        self.m_oCfg = oCfg
        #background image for plotting results
        self.m_oImgBg = oImgBg.copy()
        self.m_bCol25ProgFlg = False
        self.m_bCol50ProgFlg = False
        self.m_bCol75ProgFlg = False
        self.m_voVyCand = []
        self.m_voLinfCand = []
        self.m_oVy = None
        self.m_fLinfSlp = None
        self.m_fLinfItcp = None
        self.m_fLinfCorr = None
        self.m_oVr = np.array([(oCfg.m_oFrmSz[1] - 1), 0.0])
        self.m_oVl = np.array([0.0, 0.0])
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
            camParam.setTntMat(0.0,0.0, (fCamHeight * self.m_oCfg.m_nLenUnit))
        
        camParam.calcProjMat()

        return camParam

    def tstStGrdPt(self, oStGrdPt:np.ndarray, camParam:CCamParam):
        nLftEdgX = (-(IMG_EXPN_RAT - 1.0) / 2.0) * self.m_oCfg.m_oFrmSz[1]
        nTopEdgY = (-(IMG_EXPN_RAT - 1.0) / 2.0) * self.m_oCfg.m_oFrmSz[0]
        nRgtEdgX = ((IMG_EXPN_RAT + 1.0) / 2.0) * self.m_oCfg.m_oFrmSz[1]
        nBtmEdgY = ((IMG_EXPN_RAT + 1.0) / 2.0) * self.m_oCfg.m_oFrmSz[0]
        
        o2dStPt = (-1.0, -1.0)
        o2dRNdPt = (-1.0,-1.0)
        o2dLNdPt = (-1.0,-1.0)
        o2dNdPt = (-1.0, -1.0)

        oNdGrdPt = np.empty((2,))
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

        if (((o2dStPt[1] >= nLftEdgX) and (o2dStPt[0] >= nTopEdgY) and (o2dStPt[1] < nRgtEdgX) and (o2dStPt[0] < nBtmEdgY)) and\
		((o2dRNdPt[1] >= nLftEdgX) and (o2dRNdPt[0] >= nTopEdgY) and (o2dRNdPt[1] < nRgtEdgX) and (o2dRNdPt[0] < nBtmEdgY)) and\
		((o2dLNdPt[1] >= nLftEdgX) and (o2dLNdPt[0] >= nTopEdgY) and (o2dLNdPt[1] < nRgtEdgX) and (o2dLNdPt[0] < nBtmEdgY)) and\
		((o2dNdPt[1] >= nLftEdgX) and (o2dNdPt[0] >= nTopEdgY) and (o2dNdPt[1] < nRgtEdgX) and (o2dNdPt[0] < nBtmEdgY))):
            return True
        else:
            return False

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

                    if self.tstStGrdPt(oStGrdPt, camParam):
                        bStGrdPtFlg = True
                        break
                    if voPt[i][1] > 0:
                        oStGrdPt[1] = -voPt[i][1]
                        oStGrdPt[0] = voPt[i][0]
                        if self.tstStGrdPt(oStGrdPt, camParam):
                            bStGrdPtFlg = True
                            break
                    if voPt[i][0] > 0:
                        oStGrdPt[1] = voPt[i][1]
                        oStGrdPt[0] = -voPt[i][0]
                        if self.tstStGrdPt(oStGrdPt, camParam):
                            bStGrdPtFlg=True
                            break
                    if voPt[i][1] > 0:
                        oStGrdPt[1] = -voPt[i][1]
                        oStGrdPt[0] = -voPt[i][0]
                        if self.tstStGrdPt(oStGrdPt, camParam):
                            bStGrdPtFlg = True
                            break
                    if voPt[i][1] < voPt[i][0]:
                        oStGrdPt[1] = voPt[i][0]
                        oStGrdPt[0] = voPt[i][1]
                        if self.tstStGrdPt(oStGrdPt, camParam):
                            bStGrdPtFlg = True
                            break
                    if vopt[i][1] < voPt[i][0]:
                        oStGrdPt[1] = -voPt[i][0]
                        oStGrdPt[0] = voPt[i][1]
                        if self.tstStGrdPt(oStGrdPt,camParam):
                            bStGrdPtFlg = True
                            break
                    if (voPt[i][1] < voPt[i][0]) and (0 < voPt[i][1]):
                        oStGrdPt[1] = voPt[i][0]
                        oStGrdPt[0] = -voPt[i][1]
                        if self.tstStGrdPt(oStGrdPt, camParam):
                            bStGrdPtFlg = True
                            break
                    if (voPt[i][1] < voPt[i][0]) and (0 < voPt[i][1]):
                        oStGrdPt[1] = -voPt[i][0]
                        oStGrdPt[0] = -voPt[i][1]
                        if self.tstStGrdPt(oStGrdPt, camParam):
                            bStGrdPtFlg = True
                            break
            if bStGrdPtFlg:
                break
            nMaxSumSqDist += 1

        return oStGrdPt
                    
    def initEdaParamRng(self, oVr, oVl, oPrincipalPt, camParam:CCamParam) -> SParamRng:
        sParamRng = SParamRng()
        oVrC = np.empty((2,))
        oVlC = np.empty((2,))
        oVrCRot = np.empty((2,))
        oVlCRot = np.empty((2,))

        oVrC[1] = oVr[1] - oPrincipalPt[1]
        oVrC[0] = oPrincipalPt[0] - oVr[0]
        oVlC[1] = oVl[1] - oPrincipalPt[1]
        oVlC[0] = oPrincipalPt[0] - oVl[0]

        fRoll = np.arctan2((oVrC[0] - oVlC[0]), (oVrC[1] - oVlC[1]))
        fRoll = fRoll - np.pi if fRoll > (np.pi/2.0) else fRoll
        fRoll = fRoll + np.pi if fRoll < (-np.pi/2.0) else fRoll

        oVrCRot = rotPt(oVrC, -fRoll)
        oVlCRot = rotPt(oVlC, -fRoll)

        if camParam:
            acK = camParam.m_afK
            fF = (acL[0] + ack[4]) / 2.0
        else:
            #check this
            fF = np.sqrt(-((oVrCRot[0]*oVlCRot[0]) + (oVrCRot[1]*oVlCRot[1])))
        
        if COORD_SYS_TYP == 0:
            fPitch = np.arctan2(oVrCRot[0], fF)
            fYaw = -np.arctan2(fF, (oVrCRot[1] - np.cos(fPitch)))
        elif COORD_SYS_TYP == 1:
            fPitch = -np.arctan2(oVrCRot[0], fF)
            fYaw = -np.arctan2((oVrCRot[1]*np.cos(fPitch)), fF)

        #construct ranges of camera parameters

        sParamRng.fFxMax =  camParam.m_afK[0] if camParam is not None else fF * (1.0 + EDA_RNG_F)
        sParamRng.fFxMin =  camParam.m_afK[0] if camParam is not None else fF * (1.0 - EDA_RNG_F)
        sParamRng.fFyMax =  camParam.m_afK[4] if camParam is not None else fF * (1.0 + EDA_RNG_F)
        sParamRng.fFyMin =  camParam.m_afK[4] if camParam is not None else fF * (1.0 - EDA_RNG_F)
        sParamRng.fCxMax =  camParam.m_afK[2] if camParam is not None else oPrincipalPt[1] + EDA_RNG_PRIN_PT
        sParamRng.fCxMin =  camParam.m_afK[2] if camParam is not None else oPrincipalPt[1] - EDA_RNG_PRIN_PT
        sParamRng.fCyMax =  camParam.m_afK[5] if camParam is not None else oPrincipalPt[0] + EDA_RNG_PRIN_PT
        sParamRng.fCyMin =  camParam.m_afK[5] if camParam is not None else oPrincipalPt[0] - EDA_RNG_PRIN_PT
        sParamRng.fRollMax = fRoll + deg2rad(EDA_RNG_ROT_ANG)
        sParamRng.fRollMin = fRoll - deg2rad(EDA_RNG_ROT_ANG)
        sParamRng.fPitchMax = fPitch + deg2rad(EDA_RNG_ROT_ANG)
        sParamRng.fPitchMin = fPitch - deg2rad(EDA_RNG_ROT_ANG)
        sParamRng.fYawMax = fYaw + deg2rad(EDA_RNG_ROT_ANG)
        sParamRng.fYawMin = fYaw - deg2rad(EDA_RNG_ROT_ANG)
        sParamRng.fTxMax = 0.0
        sParamRng.fTxMin = 0.0

        if COORD_SYS_TYP == 0:
            sParamRng.fTyMax = -self.m_oCfg.m_fCalCamHeiMin * self.m_oCfg.m_nLenUnit
            sParamRng.fTyMin = -self.m_oCfg.m_fCalCamHeiMax * self.m_oCfg.m_nLenUnit
            sParamRng.fTzMax = 0.0
            sParamRng.fTzMin = 0.0
        elif COORD_SYS_TYP == 1:
            sParamRng.fTyMax = 0.0
            sParamRng.fTyMin = 0.0
            sParamRng.fTzMax = self.m_oCfg.m_fCalCamHeiMax * self.m_oCfg.m_nLenUnit
            sParamRng.fTzMin = self.m_oCfg.m_fCalCamHeiMin * self.m_oCfg.m_nLenUnit
        
        return sParamRng

    def plt3dGrd(self, camParam:CCamParam, oVr, oVl, oStGrdPt):
        afK = camParam.m_afK
        afR = camParam.m_afR
        afT = camParam.m_afT
        afP = camParam.m_afP

        f = open(self.m_oCfg.m_OutCamParamPth, "w")
        f.write(f"{afK[0]},{afK[1]},{afK[2]},{afK[3]},{afK[4]},{afK[5]},{afK[6]},{afK[7]},{afK[8]}\n")
        f.write(f"{afR[0]},{afR[1]},{afR[2]},{afR[3]},{afR[4]},{afR[5]},{afR[6]},{afR[7]},{afR[8]}\n")
        f.write(f"{afT[0]},{afT[1]},{afT[2]}\n")
        f.write(f"{afK[0]},{afK[1]},{afK[2]},{afK[3]},{afK[4]},{afK[5]},{afK[6]},{afK[7]},{afK[8]},{afR[9]},{afR[10]},{afR[11]}")
        f.close()

        o2dMeasPt = np.empty((2,))
        bgImg = self.m_oImgBg.copy()
        cv2.circle(bgImg, oVr, 3, (255,128,0), 2)
        cv2.circle(bgImg, oVl, 3, (255,128,0), 2)

        oNdGrdPt = np.empty((2,), dtype=int)
        oNdGrdPt[1] = oStGrdPt[1] + self.m_oCfg.m_nCalGrdSzR
        oNdGrdPt[0] = oStGrdPt[0] + self.m_oCfg.m_nCalGrdSzL

        voMeasLnSegNdPt = self.m_oCfg.m_voCalMeasLnSegNdPt
        oSt2dPt = np.empty((2,), dtype=int)
        oNd2dPt = np.empty((2,), dtype=int)

        for i in range(len(voMeasLnSegNdPt)//2):
            oSt2dPt = voMeasLnSegNdPt[i*2]
            oNd2dPt = voMeasLnSegNdPt[i*2+1]
            cv2.line(bgImg, oSt2dPt, oNd2dPt, (0,255,0), 3)
        
        for iL in range(oStGrdPt[0], oNdGrdPt[0]):
            for iR in range(oStGrdPt[1], oStGrdPt[1]):
                if COORD_SYS_TYP == 0:
                    o2dMeasPt = proj3d22d(np.array([iR, 0.0, iL]), afP, self.m_oCfg.m_nLenUnit)
                elif COORD_SYS_TYP == 1:
                    o2dMeasPt = proj3d22d(np.array([iR, iL, 0.0]), afP, self.m_oCfg.m_nLenUnit)

                if (0 <= o2dMeasPt[1]) and (bgImg.shape[1] > o2dMeasPt[1]) and\
                   (0 <= o2dMeasPt[0]) and (bgImg.shape[0] > o2dMeasPt[0]):
                    cv2.circle(bgImg, o2dMeasPt, 3, (0,0,255), 10)
        cv2.namedWindow("3D grid on ground plane", cv2.WINDOW_NORMAL)
        cv2.imshow("3D grid on ground plane", bgImg)
        cv2.waitkey(1)

    def calcReprojErr(self, camParam:CCamParam) -> float:
        voMeasLnSegNdPt = self.m_oCfg.m_voCalMeasLnSegNdPt
        vfMeasLnSegDist = self.m_oCfg.m_vfCalMeasLnSegDist
        fReprojErr = 0.0

        oSt2dPt = np.empty((2,))
        oNd2DPt = np.empty((2,))
        oSt3dPt = np.empty((3,))
        oNd3dPt = np.empty((3,))

        for i in range(len(voMeasLnSegNdPt)/2):
            oSt2dPt = voMeasLnSegNdPt[i*2]
            oNd2DPt = voMeasLnSegNdPt[i*2+1]
            oSt3dPt = bkproj2d23d(oSt2dPt, camParam.m_afK, self.m_oCfg.m_nLenUnit)
            oNd3dPt = bkproj2d23d(oNd2DPt, camParam.m_afK, self.m_oCfg.m_nLenUnit)

            fReprojErr += np.abs(np.linalg.norm(oNd3dPt - oSt3dPt) - vfMeasLnSegDist[i])

        return fReprojErr

    def calCamEdaOpt(self):

        nR = EDA_INIT_POP
        nN = EDA_SEL_POP
        nIterNum = EDA_ITER_NUM
        iIter = 0
        oCamParamRand = CCamParam()
        oCamParam = CCamParam()

        #set Starting Grid Point
        camParam = self.compCamParam(self.m_oVr, self.m_oVl, self.m_oPrinPt, ((self.m_oCfg.m_fCalCamHeiMax + self.m_oCfg.m_fCalCamHeiMin)/ 2.0))
        oStGrdPt = self.calcStGrdPt(camParam)

        sParamRng = initEdaParamRng(self.m_oVr, self.m_oVl, self.m_oPrinPt)

        # EDA optimization

        if nN > nR:
            raise Exception("Error: Selected population should be less than initial population")

        voCamParam = []

        for iR in range(nR):
            oCamParamRand.initCamMdl(sParamRng)
            voCamParam.append(oCamParamRand)
        
        print("Start EDA optimization for camera calibration")

        while nIterNum > iIter:
            print(f"==== generation {iIter}: ====")
            iProc = 0
            bProc25 = False
            bProc50 = False
            bProc75 = False
            fReprojErrMean = 0.0
            fReprojErrStd = 0.0

            for ivoCamParam in voCamParam:
                fFx = ivoCamParam.m_fFx
                fFy = ivoCamParam.m_fFy
                fCx = ivoCamParam.m_fCx
                fCy = ivoCamParam.m_fCy
                fRoll = ivoCamParam.m_fRoll
                fPitch = ivoCamParam.m_fPitch
                fYaw = ivoCamParam.m_fYaw
                fTx = ivoCamParam.m_fTx
                fTy = ivoCamParam.m_fTy
                fTz = ivoCamParam.m_fTz

                oCamParam.setInParamMat(fFx, fFy, fCx, fCy)
                oCamParam.setRotMat(fRoll, fPitch, fYaw)
                oCamParam.setTntMat(fTx, fTy, fTz)
                oCamParam.calcProjMat()

                fReprojErr = self.calcReprojErr(oCamParam)
                ivoCamParam.m_fReprojErr = fReprojErr
                fReprojErrMean += fReprojErr
                iProc += 1

                if ((float(iProc) / float(nR)) > 0.25) and (not bProc25):
                    print("25%%...")
                    bProc25 = True
                if ((float(iProc) / float(nR)) > 0.50) and (not bProc50):
                    print("50%%...")
                    bProc50 = True
                if ((float(iProc) / float(nR)) > 0.75) and (not bProc75):
                    print("75%%...")
                    bProc75 = True

            fReprojErrMean /= nR

            for ivoCamParam in voCamParam:
                fReprojErr = ivoCamParam.m_fReprojErr
                fReprojErrStd += (fReprojErr - fReprojErrMean) * (fReprojErr - fReprojErrMean)

            fReprojErrStd = np.sqrt(fReprojErrStd / nR)

            print("100%%!")
            print(f"current error mean = {fReprojErrMean}")
            print(f"current error standard deviation = {fReprojErrStd}")

            if not fReprojErr:
                print("calibration Failed")
                break
        
            #check if generaiton needs to stop
            if (o < iIter) and ((fReprojErrMeanPrev * EDA_REPROJ_ERR_THLD) > np.abs(fReprojErrMean - fReprojErrMeanPrev)):
                print("Reprojection error is small enough. Stop generation.")
                break

            #how to initialize this value for the first time?
            ReprojErrMeanPrev = fReprojErrMean

            voCamParam = sorted(voCamParam, key=lambda param: param.m_fReprojErr)
            voCamParam = voCamParam[:nN]

            for iR in range(nR):
                oCamParamRand = CCamParam()
                oCamParamRand.initCamMdl(sParamRng)
                voCamParam.append(oCamParamRand)
            
            iIter += 1
            print("\n")
            
        if nIterNum <= iIter:
            print("Exit: results cannot converge")