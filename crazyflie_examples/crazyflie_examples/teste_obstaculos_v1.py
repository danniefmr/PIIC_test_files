import sys
import pygame
from pygame.locals import *

from pathlib import Path
from crazyflie_py import Crazyswarm
import numpy as np
        
def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    CFs = allcfs.crazyflies
    pos = np.array(CFs[0].initialPosition)
    #pos = pos + np.array([0,0,0.8])
    pos2 = pos
    clock_speed = 4 # Hz

    pygame.init()
    clock = pygame.time.Clock()

    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    for joystick in joysticks:
        print(joystick.get_name())

    varx = 0
    vary = 0
    varz = 0
    varyaw = 0
    yaw = np.pi/180*0
    flag = 2
    flag_pilar = False

    #objeto 1 - mesa
    prop1 = SimpleProp(0.61, 0.65, 0.82)
    prop1.coordinates(0.0, 0.60, 0.0, pos[0], pos[1], pos[2]) #left center

    #objeto 2
    #prop1.coordinates(1.44,0.76, 0.0, pos[0], pos[1], pos[2]) # center
    #prop2 = SimpleProp(0.88, 0.09, 2.09)
    #prop2.coordinates(1.3, 1.15, 0.0, pos[0], pos[1], pos[2])


    while True:
   
        for event in pygame.event.get():

            if event.type == JOYBUTTONDOWN:
                if event.button == 0:
                    pos = pos + np.array([0, 0, 0.4])
                    pos2 = pos
                    CFs[0].takeoff(targetHeight= pos[2], duration=2.0)
                    flag = -2*clock_speed

                elif event.button == 2:
                    CFs[0].land(targetHeight=0.06, duration=2.0 + pos[2])
                    pos[2] = 0
                    flag = -2*clock_speed
                    yaw = 0
   
            if event.type == JOYAXISMOTION:
                if event.axis == 3:
                    vary = event.value
                if event.axis == 4:
                    varx = event.value
                if event.axis == 1:
                    varz = event.value
                if event.axis == 0:
                    varyaw = event.value
            
            if event.type == JOYDEVICEADDED:
                joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
                for joystick in joysticks:
                    print(joystick.get_name())
            if event.type == JOYDEVICEREMOVED:
                joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    sys.exit()

        if pos[2] != 0:
            print("posicao anterior", pos)
            pos2 = pos + np.array([(varx*0.06*np.cos(yaw)*(-1) + vary*0.06*np.sin(yaw))*10/clock_speed, (varx*0.06*np.sin(yaw)*(-1) + vary*0.06*np.cos(yaw)*(-1))*10/clock_speed, (varz*0.06)*(-1)*10/clock_speed])
            print("posicao posterior", pos2)
            print(" ")
            yaw += varyaw * np.pi/180*90/20*(-1)*10/clock_speed

        #************** limites da arena *****************
        if pos2[1] > 1.5: #left
            pos2[1] = 1.5
            print("left limit")
        if pos2[1] < -1.0: #ritgh
            pos2[1] = -1.0
            print("rigth limit")
        if pos2[0] > 1.0: #front
            pos2[0] = 1.0
            print("front limit")
        if pos2[0] < -0.8: #back
            pos2[0] = -0.8
            print("back limit")
        if pos2[2] > 2.0: #up
            pos2[2] = 2.0
            print("height limit")
        if pos2[2] < 0.15 and flag == 1: #down
            pos2[2] = 0.15
        
        

        if flag < 1 and pos[2] != 0:
            flag+=1
        #print("flag: ",flag)
        #print(pos)
        #print("yaw = ",yaw*180/np.pi)
        if(flag == 1 and pos[2] >= 0.2):
            if (prop1.avoidProp(pos2[0], pos2[1], pos2[2]) ): #prop1.avoidProp(pos2[0], pos2[1], pos2[2])
                pos = pos2
                CFs[0].goTo(pos, yaw, 1.5)
                #last_position = pos
            else:
                print("cosision_detected")
        
        
        

        clock.tick(clock_speed)