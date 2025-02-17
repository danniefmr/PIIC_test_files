#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np

def main():
    
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # # enable logging to SD card if present (assuming disabled before)
    # cf.setParam('usd.logging',1)
    # timeHelper.sleep(1.0)

    # do absolutely nothing for ?? seconds
    dt = 60
    print("Standing still for ",dt," seconds.")
    timeHelper.sleep(dt)

    # # disable logging to SD card if present (assuming enabled before)
    # cf.setParam('usd.logging',0)
    # timeHelper.sleep(1.0)


if __name__ == '__main__':
    main()
