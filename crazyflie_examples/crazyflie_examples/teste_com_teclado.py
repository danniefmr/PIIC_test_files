#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
from math import sqrt


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    CFs = allcfs.crazyflies

    #traj1 = Trajectory()
    #traj1.loadcsv(Path(__file__).parent / 'data/figure8.csv')

    TRIALS = 1
    TIMESCALE = 1.0
    nCFs = len(CFs)
    #for i in range(TRIALS):
        #for cf in allcfs.crazyflies:
            #cf.uploadTrajectory(0, 0, traj1)

        #allcfs.takeoff(targetHeight=0.5, duration=2.0)


    for iCF in range(nCFs):
        keyboard = input("Order: ")

        pos = np.array(CFs[iCF].initialPosition)

        while(keyboard != 'x'):
        
            if(keyboard == 't'): #takeoff
                CFs[iCF].takeoff(targetHeight=0.6, duration=2.0)
                pos = pos + np.array([0, 0, 0.6])
            
                print(pos)
                timeHelper.sleep(3.0) 

            elif(keyboard == 'l'): #land
                CFs[iCF].land(targetHeight=0.06, duration=2.0)
                pos[2] = 0
                print(pos)
                timeHelper.sleep(3.0)

            elif(keyboard == 'w'): #front
                if(pos[0] == 1.0):
                    print("Reach the safe limit")
                else:
                    pos = pos + np.array([0.5, 0, 0])
                    print(pos)
                    CFs[iCF].goTo(pos, 0, 2.0)
                timeHelper.sleep(0.5)

            elif(keyboard == 's'): #backward
                if(pos[0] == -0.5):
                    print("Reach the safe limit")
                else:
                    pos = pos - np.array([0.5, 0, 0])
                    print(pos)
                    CFs[iCF].goTo(pos, 0, 2.0)
                timeHelper.sleep(0.5)

            elif(keyboard == 'a'): #left
                if(pos[1] == 1.5):
                    print("Reach the safe limit")
                else:
                    pos = pos + np.array([0, 0.5, 0])
                    print(pos)
                    CFs[iCF].goTo(pos, 0, 2.0)
                timeHelper.sleep(0.5)

            elif(keyboard == 'd'): #right
                if(pos[1] == -1.0):
                    print("Reach the safe limit")
                else:
                    pos = pos - np.array([0, 0.5, 0])
                    print(pos)
                    CFs[iCF].goTo(pos, 0, 2.0)
                timeHelper.sleep(0.5)

            elif(keyboard == 'r'): #up
                if(pos[2] == 1.5):
                    print("Reach the safe limit")
                    timeHelper.sleep(0.5)
                else:
                    pos = pos + np.array([0, 0, 0.5])
                    print(pos)
                    CFs[iCF].goTo(pos, 0, 2.0)
                    timeHelper.sleep(3)

            elif(keyboard == 'f'): #down
                pos = pos - np.array([0, 0, 0.5])
                print(pos)
                CFs[iCF].goTo(pos, 0, 2.0)
                timeHelper.sleep(3)

            elif(keyboard == 'o'): #back to the origin

                dx = pos[0] - CFs[iCF].initialPosition[0]
                dy = pos[1] - CFs[iCF].initialPosition[1]
                dist = sqrt(dx*dx+dy*dy)
                dT = dist*4

                pos[0] = CFs[iCF].initialPosition[0]
                pos[1] = CFs[iCF].initialPosition[1]
                CFs[iCF].goTo(pos, 0, dT)
                timeHelper.sleep(0.5)
            
            keyboard = input("Order: ")

        if(pos[2] != 0): 
            timeHelper.sleep(5.0) 
            allcfs.land(targetHeight=0.06, duration=2.0)
            timeHelper.sleep(3.0)   

    timeHelper.sleep(2.0)



if __name__ == '__main__':
    main()
