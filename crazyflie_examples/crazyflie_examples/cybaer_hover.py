#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np

def main():

    Z = 1.0
    #dX = 0.6*0;
    #dY = -0.6*0;
    #dZ = -0.4*0;
    dX = 1.0;
    dY = -1.0*0;
    dZ = 1.0*0;
    dYaw = np.pi/2.0*0;
    dT = 2.0;
    nreps = 3;
    
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # # enable logging to SD card if present (assuming disabled before)
    # cf.setParam('usd.logging',1)
    # timeHelper.sleep(1.0)

    cf.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2+Z)
    
    # (absolute) x-axis position control
    pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
    cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(3+Z)

    # timeHelper.sleep(20)
    
    for irep in range(nreps):
        pos = np.array(cf.initialPosition) + np.array([dX, dY, Z+dZ])
        cf.goTo(pos, dYaw, dT)
        timeHelper.sleep(3+Z)
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, 0, dT)
        timeHelper.sleep(3+Z)
    
    cf.land(targetHeight=0.04, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)

    # # disable logging to SD card if present (assuming enabled before)
    # cf.setParam('usd.logging',0)
    # timeHelper.sleep(1.0)


if __name__ == '__main__':
    main()
