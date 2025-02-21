import sys
import pygame
from pygame.locals import *

from pathlib import Path
from crazyflie_py import Crazyswarm

import numpy as np


################################################################################################################
#                   Script para o drone realizar uma trajetória a contornar o escadote                         #
#                                                                                                              #
#                                     Coordenadas dos objetos dentro da arena:                                 #
#                   Mesa - coordenadas do pé inferior direito da mesa: (-0.14; 1.15))                          #
#                   Escadote - coordenadas do pé inferior direito - (-0.25; 1.32)                              #
#                                                                                                              #
#                                           Trajetória:                                                        #
#                                                                                                              #
#                 O drone dá take off em cima da mesa e dirige-se para a traseira do escadote                  #
#                 e de seguida começa a contornar os seus degraus e pousa no chão                              #
#                                                                                                              #
################################################################################################################

        
def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    CFs = allcfs.crazyflies
    pos = np.array(CFs[0].initialPosition)
    pos = pos + np.array([0.14,1.24,0.82])

    #take off
    allcfs.takeoff(targetHeight=1.0, duration=1.0)
    timeHelper.sleep(4.0)

    #
    pos = np.array([-0.3, -0.78, 1.49])
    CFs[0].goTo(pos, 0, 3.0)
    timeHelper.sleep(3.0)

    #
    pos = np.array([0.4, -0.78, 1.49])
    CFs[0].goTo(pos, 0, 2.0)
    timeHelper.sleep(1.5)

    # 1º degrau
    pos = np.array([0.4, -0.78, 1.1])
    CFs[0].goTo(pos, 0, 1.5)
    timeHelper.sleep(1.5)

    # 1º degrau (dentro do escadote)
    pos = np.array([0.11, -0.78, 1.1])
    CFs[0].goTo(pos, 0, 1.5)
    timeHelper.sleep(1.5)

    # 2º degrau
    pos = np.array([0.11, -0.78, 0.88])
    CFs[0].goTo(pos, 0, 1.5)
    timeHelper.sleep(1.5)

    # 2º degrau (fora do escadote)
    pos = np.array([0.58, -0.78, 0.88])
    CFs[0].goTo(pos, 0, 1.5)
    timeHelper.sleep(1.5)

    # 3º degrau
    pos = np.array([0.58, -0.78, 0.65])
    CFs[0].goTo(pos, 0, 1.5)
    timeHelper.sleep(1.5)

    # 3º degrau (dentro do escadote)
    pos = np.array([0.11, -0.78, 0.65])
    CFs[0].goTo(pos, 0, 1.5)
    timeHelper.sleep(1.5)

    # 4º degrau
    pos = np.array([0.11, -0.78, 0.46])
    CFs[0].goTo(pos, 0, 1.5)
    timeHelper.sleep(1.5)

    # 4º degrau (fora do escadote)
    pos = np.array([0.8, -0.78, 0.46])
    CFs[0].goTo(pos, 0, 1.5)
    timeHelper.sleep(1.5)

    # 5º degrau
    pos = np.array([0.8, -0.78, 0.25])
    CFs[0].goTo(pos, 0, 1.5)
    timeHelper.sleep(1.5)

    # 5º degrau (dentro do escadote)
    pos = np.array([0.11, -0.78, 0.25])
    CFs[0].goTo(pos, 0, 1.5)
    timeHelper.sleep(1.5)

    allcfs.land(targetHeight=0.06, duration=1.5)
    timeHelper.sleep(3.0)

  