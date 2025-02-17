import sys
import pygame
from pygame.locals import *

from pathlib import Path
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    CFs = allcfs.crazyflies
    pos = np.array(CFs[0].initialPosition)
    clock_speed = 4 # Hz

    pygame.init()
    #pygame.display.set_caption('game base')
    #screen = pygame.display.set_mode((500, 500), 0, 32)
    clock = pygame.time.Clock()

    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    for joystick in joysticks:
        print(joystick.get_name())

    #my_square = pygame.Rect(50, 50, 50, 50)
    #my_square_color = 0
    #colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    #motion = [0, 0]
    #j = 0
    varx = 0
    vary = 0
    varz = 0
    varyaw = 0
    yaw = np.pi/180*0
    flag = 2
    while True:
        #j+=1
        #print("iteration: ", j)
        #screen.fill((0, 0, 0))

        #pygame.draw.rect(screen, colors[my_square_color], my_square)
        #if abs(motion[0]) < 0.1:
        #    motion[0] = 0
        #if abs(motion[1]) < 0.1:
        #    motion[1] = 0
        #my_square.x += motion[0] * 10
        #my_square.y += motion[1] * 10

    
        for event in pygame.event.get():

            #print(event)
            if event.type == JOYBUTTONDOWN:
                #print(event)
                if event.button == 0:
                    CFs[0].takeoff(targetHeight=0.6, duration=2.0)
                    pos = pos + np.array([0, 0, 0.6])
                    flag = -2*clock_speed
                    print(pos)
                    #timeHelper.sleep(2.0)

                elif event.button == 2:
                    CFs[0].land(targetHeight=0.06, duration=2.0 + pos[2])
                    pos[2] = 0
                    flag = -2*clock_speed
                    yaw = 0
                    print(pos)
                    #timeHelper.sleep(2.0)
                    #my_square_color = (my_square_color + 1) % len(colors)
            #if event.type == JOYBUTTONUP:
                #print(event)
            if event.type == JOYAXISMOTION:
                #print(event)
                if event.axis == 0:
                    vary = event.value
                if event.axis == 1:
                    varx = event.value
                if event.axis == 4:
                    varz = event.value
                if event.axis == 3:
                    varyaw = event.value
                
                #if event.axis == 0: 
                #    print(pos)
                 #   CFs[0].goTo(pos, 0, 2.0)
                  #  timeHelper.sleep(0.5)
                    #motion[event.axis] = event.value
            #if event.type == JOYHATMOTION:
                #print(event)
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
            pos = pos + np.array([(varx*0.06*np.cos(yaw)*(-1) + vary*0.06*np.sin(yaw))*10/clock_speed, (varx*0.06*np.sin(yaw)*(-1) + vary*0.06*np.cos(yaw)*(-1))*10/clock_speed, (varz*0.06)*(-1)*10/clock_speed])
            yaw += varyaw * np.pi/180*90/20*(-1)*10/clock_speed
        #************** limites da arena *****************
        if pos[1] > 1.5: #left
            pos[1] = 1.5
            print("left limit")
        if pos[1] < -1.0: #ritgh
            pos[1] = -1.0
            print("rigth limit")
        if pos[0] > 1.0: #front
            pos[0] = 1.0
            print("front limit")
        if pos[0] < -0.5: #back
            pos[0] = -0.5
            print("back limit")
        if pos[2] > 2.0: #up
            pos[2] = 2.0
            print("height limit")
        if pos[2] < 0.4 and flag == 1: #down
            pos[2] = 0.4
        #************** **************** *****************
        

        if flag < 1 and pos[2] != 0:
            flag+=1
        #print("flag: ",flag)
        print(pos)
        print("yaw = ",yaw*180/np.pi)
        if(flag == 1 and pos[2] >= 0.2):
            CFs[0].goTo(pos, yaw, 1.5)
        

        #pygame.display.update()
        clock.tick(clock_speed)