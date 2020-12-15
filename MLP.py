import numpy as np
import sklearn
import cv2

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim


class MLP(nn.Module):
    def __init__(self, hiddenSpec: list = None, inShape: int = 3):
        super(Net, self).__init__()
        self.inShape = inshape
        self.numLayers = len(hiddenSpec)
        self.hiddenSpec = hiddenSpec
        self.layers = []

        self.dense1 = nn.Linear(self.inShape, 
                                self.hiddenSpec[0])
        for i in range(1, self.numLayers):
            self.layers.append(nn.Linear(self.hiddenSpec[i-1],
                                         self.hiddenSpec[i])   
        self.out = nn.Linear(self.hiddenSpec[-1], 1) # output corresponding to real world measure

    def forward(self, x):
        x = F.relu(self.dense1(x))
        for hiddenLayer in self.layers:
            x = F.relu(hiddenLayer(x))
        x = self.out(x)

        return x


class MLPRegressor(object):
    def __init__(self, 
                 hiddenSpec: list = None, 
                 inShape: int = 3, 
                 lossFunc=nn.MSELoss(), 
                 optimizer=optim.SGD,
                 learningRate=0.01):

        self.net = MLP(hiddenSpec,inshape)
        self.lossFunc = lossFunc
        self.optimizer=optimizer(net.parameters(), learningRate)