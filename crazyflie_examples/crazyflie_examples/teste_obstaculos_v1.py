import sys
import pygame
from pygame.locals import *

from pathlib import Path
from crazyflie_py import Crazyswarm
from crazyflie_py.object_avoiding import SimpleProp

#import object_avoiding

#from crazyflie_py.uav_trajectory import Trajectory
import numpy as np



'''def box(x, y, safe_dis, pos):
    vertex1 = [x + safe_dis, y - safe_dis]
    vertex2 = [x + safe_dis, y + safe_dis]
    vertex3 = [x - safe_dis, y + safe_dis]
    vertex4 = [x - safe_dis, y - safe_dis]

    if pos[0] < vertex1[0] and pos[1] < vertex1[1] and pos[1] > vertex2[1] and pos[0] > vertex3[0]:
        return True
    else:
        return False'''

        
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
    flag_pilar = False

    prop1 = SimpleProp(0.61, 0.51, 0.82)
    prop1.coordinates(0.0, 0.60, 0.0, pos[0], pos[1], pos[2]) #left center
    #prop1.coordinates(1.44,0.76, 0.0, pos[0], pos[1], pos[2]) # center
    prop2 = SimpleProp(0.88, 0.09, 2.09)
    prop2.coordinates(1.3, 1.15, 0.0, pos[0], pos[1], pos[2])


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
                    pos = pos + np.array([0, 0, 0.4])
                    pos2 = pos
                    CFs[0].takeoff(targetHeight= pos[2], duration=2.0)
                    flag = -2*clock_speed
                    #print(pos)
                    #timeHelper.sleep(2.0)

                elif event.button == 2:
                    CFs[0].land(targetHeight=0.06, duration=2.0 + pos[2])
                    pos[2] = 0
                    flag = -2*clock_speed
                    yaw = 0
                    #print(pos)
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
            if (prop1.avoidProp(pos2[0], pos2[1], pos2[2]) and prop2.avoidProp(pos2[0], pos2[1], pos2[2])): #prop1.avoidProp(pos2[0], pos2[1], pos2[2])
                pos = pos2
                CFs[0].goTo(pos, yaw, 1.5)
                #last_position = pos
            else:
                print("cosision_detected")
        
        
        

        #pygame.display.update()
        clock.tick(clock_speed)