#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np

def main():

    Z = 1.0
    dX = 0.5
    dY = -0.4
    dZ = 0.4
    dYaw = np.pi/2.0*0
    dT = 2 
    dTsmall = 0.5
    dTwait = 5.0 
    nreps = 0
    
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    CFs = allcfs.crazyflies
    nCFs = len(CFs)
    
    print('Number of CFs: ',nCFs)
    for iCF in range(nCFs):
        print('Drone ', iCF,'esta na altura : ',Z + 0.2*iCF)

    # # enable logging to SD card if present (assuming disabled before)
    # cf.setParam('usd.logging',1)
    # timeHelper.sleep(1.0)
    print(dYaw)
    # allcfs.takeoff(targetHeight=Zb, duration=1.0+Zb)
    for iCF in range(nCFs):
        CFs[iCF].takeoff(targetHeight=Z+0.2*iCF-0.4, duration=1.0+Z)
        timeHelper.sleep(dTsmall)
    timeHelper.sleep(2)
    
    # (absolute) x-axis position control
    for iCF in range(nCFs):
        posi = np.array(CFs[iCF].initialPosition) + np.array([0,0,Z+0.2*iCF])
        
        CFs[iCF].goTo(posi, 0, dT)
        timeHelper.sleep(dTsmall)
        ##
        timeHelper.sleep(2)
        CFs[iCF].goTo(posi, np.pi/2.0*3, 1.5)
        timeHelper.sleep(2)
        #CFs[iCF].goTo(posi, np.pi/2.0*1, 1.5)
        ##
    timeHelper.sleep(2)

    # timeHelper.sleep(10)
    
    for irep in range(nreps):
        for iCF in range(nCFs):
            posi = np.array(CFs[iCF].initialPosition) + np.array([dX,0,Z+0.2*iCF])
            CFs[iCF].goTo(posi, dYaw, dT)
            timeHelper.sleep(dTsmall)
        timeHelper.sleep(dTwait)
        dYaw = np.pi/2.0*1 ##
        for iCF in range(nCFs):
            posi = np.array(CFs[iCF].initialPosition) + np.array([dX,dY,Z+0.2*iCF+dZ])
            CFs[iCF].goTo(posi, dYaw, dT)
            timeHelper.sleep(dTsmall)
        timeHelper.sleep(dTwait)
        dYaw = np.pi/2.0*2 ##
        for iCF in range(nCFs):
            posi = np.array(CFs[iCF].initialPosition) + np.array([0,dY,Z+0.2*iCF+dZ])
            CFs[iCF].goTo(posi, dYaw, dT)
            timeHelper.sleep(dTsmall)
        timeHelper.sleep(dTwait)
        dYaw = np.pi/2.0*3 ##
        for iCF in range(nCFs):
            posi = np.array(CFs[iCF].initialPosition) + np.array([0,0,Z+0.2*iCF])
            CFs[iCF].goTo(posi, 0, dT)
            timeHelper.sleep(dTsmall)
        timeHelper.sleep(dTwait)
        dYaw = np.pi/2.0*4 ##
    
    # allcfs.land(targetHeight=0.02, duration=1.0+dT)
    for iCF in range(nCFs):
        CFs[iCF].land(targetHeight=0.02, duration=1.0+Z)
        timeHelper.sleep(dTsmall)
    timeHelper.sleep(2)

    # # disable logging to SD card if present (assuming enabled before)
    # cf.setParam('usd.logging',0)
    # timeHelper.sleep(1.0)


if __name__ == '__main__':
    main()
