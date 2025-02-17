#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np

def executeInfTrajectory(timeHelper, cf, traj, rate=100, offset=np.zeros(3), nloops=1):
    
    start_time = timeHelper.time()
    for i_loop in range(nloops):
        status = cf.get_status()
        print(f'pm state : {status["pm_state"]}, battery: {status["battery"]}')
        if not status['pm_state'] == 0:
            break
        
        # pose = cf.get_pose()
        # print("pose = ",pose)
        # print("status = ",status)
        # print(f'pos= [{pose["px"]},{pose["py"]},{pose["pz"]}], quat = [{pose["qx"]},{pose["qy"]},{pose["qz"]},{pose["qw"]}].')

        print(f'Trajectory loop {i_loop+1} of {nloops}.')

        while not timeHelper.isShutdown():
            t_total = timeHelper.time() - start_time
            if t_total > (i_loop+1)*traj.duration:
                break

            t = t_total % traj.duration
            e = traj.eval(t)
            cf.cmdFullState(
                e.pos + np.array(cf.initialPosition) + offset,
                e.vel,
                e.acc,
                e.yaw,
                e.omega)
            
            timeHelper.sleepForRate(rate)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    
    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / 'data/figure8.csv')

    rate = 30.0
    Z = 1.0
    Nloops = 2
    offset=np.array([0, 0, Z])

    # # enable logging to SD card if present (assuming disabled before)
    # cf.setParam('usd.logging',1)
    # timeHelper.sleep(1.0)

    # takeoff and go to initial position
    print('takeoff')
    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)
    pos = np.array(cf.initialPosition) + offset
    cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(Z+2.0)

    # fly traj until battery is low or Nloops times
    executeInfTrajectory(timeHelper, cf, traj1, rate, offset, Nloops)
    
    print('landing')
    cf.notifySetpointsStop()
    pos = np.array(cf.initialPosition) + offset
    cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(Z+2.0)
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    # # disable logging to SD card if present (assuming enabled before)
    # cf.setParam('usd.logging',0)
    # timeHelper.sleep(1.0)


if __name__ == '__main__':
    main()
