# -*- coding: utf-8 -*-
"""
Created on Fri Aug 19 13:27:39 2016

@author: cunnia3
"""

import numpy as np
from DMPLib import DMP
import matplotlib.pyplot as plt
import csv

class DMPTrainer:
    def __init__(self, csvName = "dmpResult1.csv"):
        self.csvName = csvName        
        
        # CSV columns = [ time | x | y | z | roll | pitch | yaw ]
        self.trajectory = np.genfromtxt('recording1.csv',delimiter=',', \
            usecols = (0, 1, 2, 3, 4, 5 ,6),dtype=np.float)
        
        self.exampleTime = self.trajectory[:,0] - self.trajectory[0,0]
        self.exampleX = self.trajectory[:,1]
        self.exampleY = self.trajectory[:,2]
        self.exampleZ = self.trajectory[:,3]
        self.exampleRoll = self.trajectory[:,4]
        self.examplePitch = self.trajectory[:,5]
        self.exampleYaw = self.trajectory[:,6]
        
        self.dmpX= DMP(500)
        self.dmpY= DMP(500)
        self.dmpZ= DMP(500)
        self.dmpRoll= DMP(500)
        self.dmpPitch= DMP(500)
        self.dmpYaw= DMP(500)
        
    def train(self):
        self.dmpX.setExample(self.exampleX,self.exampleTime)
        self.dmpY.setExample(self.exampleY,self.exampleTime)
        self.dmpZ.setExample(self.exampleZ,self.exampleTime)
        self.dmpRoll.setExample(self.exampleRoll,self.exampleTime)
        self.dmpPitch.setExample(self.examplePitch,self.exampleTime)
        self.dmpYaw.setExample(self.exampleYaw,self.exampleTime)
        
        self.dmpX.imitate()
        self.dmpY.imitate()
        self.dmpZ.imitate()
        self.dmpRoll.imitate()
        self.dmpPitch.imitate()
        self.dmpYaw.imitate()
        
    def simulateDMP(self):
        duration = self.exampleTime[-1]
        self.dmpX.run(duration)
        self.dmpY.run(duration)
        self.dmpZ.run(duration)
        
        self.dmpRoll.run(duration)
        self.dmpPitch.run(duration)
        self.dmpYaw.run(duration)
        
        self.resultX = self.dmpX.responsePos
        self.resultY = self.dmpY.responsePos
        self.resultZ = self.dmpZ.responsePos
        self.resultRoll = self.dmpRoll.responsePos
        self.resultPitch = self.dmpPitch.responsePos
        self.resultYaw = self.dmpYaw.responsePos
        
    def plot(self):
        plt.figure()
        plt.title('Original and scaled versions')
        plt.plot(-self.exampleX,-self.exampleY,'g-')
        plt.plot(-self.resultX,-self.resultY,'r-')
        plt.show()
        
    def writeCSV(self):
        myfile = open(self.csvName, 'wb')
        wr = csv.writer(myfile)
        for i in range(len(self.exampleX)):
            poseInstance = [self.exampleX[i],self.exampleY[i],self.exampleZ[i],\
            self.exampleRoll[i],self.examplePitch[i],self.exampleYaw[i]]
            wr.writerow(poseInstance)
            
    def runTrainer(self):
        self.train()
        self.simulateDMP()
        self.plot()
        self.writeCSV()
        
myTrainer = DMPTrainer()
myTrainer.runTrainer()
