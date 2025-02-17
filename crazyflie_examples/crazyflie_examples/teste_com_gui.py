from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from tkinter import *
from pathlib import Path
import numpy as np
from math import sqrt

def button_click_up():
    print("A ir pra cima")
    global pos
    global CFs
    global timeHelper

    if(pos[2] >= 1.5):
        print("Reach the safe limit")
        timeHelper.sleep(0.5)
    else:
        pos = pos + np.array([0, 0, 0.5])
        print(pos)
        CFs[0].goTo(pos, 0, 2.0)
        timeHelper.sleep(3)

def button_click_down():
    global pos
    global CFs
    global timeHelper

    print("A ir pra baixo")
    pos = pos - np.array([0, 0, 0.5])
    print(pos)
    CFs[0].goTo(pos, 0, 2.0)
    timeHelper.sleep(3)

def button_click_up_arrow(slider_vel):
    global pos
    global CFs
    global timeHelper
    i = 6-slider_vel
    if(pos[0] >= 1.0):
        print("Reach the safe limit")
    else:
        pos = pos + np.array([0.5, 0, 0])
        print(pos)
        CFs[0].goTo(pos, 0, 0.5*i)
    timeHelper.sleep(0.5)
    

def button_click_down_arrow(slider_vel):
    print("A ir pra tras")
    global pos
    global CFs
    global timeHelper
    i = 6-slider_vel
    if(pos[0] <= -0.5):
        print("Reach the safe limit")
    else:
        pos = pos - np.array([0.5, 0, 0])
        print(pos)
        CFs[0].goTo(pos, 0, 0.5*i)
    timeHelper.sleep(0.5)
    

def button_click_left_arrow(slider_vel):
    print("A ir pra esquerda")
    global pos
    global CFs
    global timeHelper
    i = 6-slider_vel
    if(pos[1] >= 1.5):
        print("Reach the safe limit")
    else:
        pos = pos + np.array([0, 0.5, 0])
        print(pos)
        CFs[0].goTo(pos, 0, 0.5*i)
    timeHelper.sleep(0.5)

def button_click_right_arrow(slider_vel):
    global pos
    global CFs
    global timeHelper
    i = 6-slider_vel
    if(pos[1] <= -1.0):
        print("Reach the safe limit")
    else:
        pos = pos - np.array([0, 0.5, 0])
        print(pos)
        CFs[0].goTo(pos, 0, 0.5*i)
    timeHelper.sleep(0.5)


def button_click_emergency():
    print("Emergency Stop")
    #continuar aqui

def button_click_land():
    print("A aterrar")
    global pos
    global CFs
    global timeHelper

    CFs[0].land(targetHeight=0.06, duration=3.0)
    pos[2] = 0
    print(pos)
    timeHelper.sleep(3.0)

def button_click_takeoff():
    print("A descolar")
    global pos
    global CFs
    global timeHelper

    CFs[0].takeoff(targetHeight=0.6, duration=2.0)
    pos = pos + np.array([0, 0, 0.6])
            
    timeHelper.sleep(3.0)

def button_click_center():
    print("A voltar para o centro")
    global pos
    global CFs 
    global timeHelper

    dx = pos[0] - CFs[0].initialPosition[0]
    dy = pos[1] - CFs[0].initialPosition[1]
    dist = sqrt(dx*dx+dy*dy)
    dT = dist*4

    pos[0] = CFs[0].initialPosition[0]
    pos[1] = CFs[0].initialPosition[1]
    CFs[0].goTo(pos, 0, dT)
    timeHelper.sleep(0.5)

    #continuar aqui

def button_click_pos(slider_pos_x,slider_pos_y,slider_pos_z):
    print("Going to x="+str(slider_pos_x*0.1)+" y="+str(slider_pos_y*0.1)+" z="+str(slider_pos_z*0.1))
    global pos
    global CFs 
    global timeHelper

    dx = pos[0] - slider_pos_x*0.1
    dy = pos[1] - slider_pos_y*0.1
    dz = pos[2] - slider_pos_z*0.1

    dist1 = sqrt(dx*dx+dy*dy)
    dist = sqrt(dz*dz + dist1*dist1)
    dT = dist*4
    if(dT < 1):
        dT=1
    #placa pos 


    pos[0] = slider_pos_x*0.1
    pos[1] = slider_pos_y*0.1
    pos[2] = slider_pos_z*0.1
    CFs[0].goTo(pos, 0, dT)
    timeHelper.sleep(0.5)

def button_click_middle_table():
    global pos
    global CFs 
    global timeHelper
    global in_table
    in_table = 1
    dx = pos[0] - 0
    dy = pos[1] - 0.8
    dz = pos[2] - 0.53

    dist1 = sqrt(dx*dx+dy*dy)
    dist = sqrt(dz*dz + dist1*dist1)
    dT = dist*4
    if (pos[1]<0.9):
        pos[0] = 0.1
        pos[1] = 0.8
        pos[2] = 0.57
        CFs[0].goTo(pos, 0, dT)
        timeHelper.sleep(dT)
    else:
        if(-0.5<pos[0]<-0.14):
            pos[0] = -0.3
            pos[1] = 1.2
            pos[2] = 0.57
        elif(0.3<pos[0]<1):
            pos[0] = 0.5
            pos[1] = 1.2
            pos[2] = 0.57
        elif(0.8<pos[2]):
            pos[0] = 0.1
            pos[1] = 0.8
            CFs[0].goTo(pos, 0, dT)
            timeHelper.sleep(dT)
            pos[2] = 0.57
        CFs[0].goTo(pos, 0, dT)
        timeHelper.sleep(dT)
    pos[0] = 0.1
    pos[1] = 1.15
    CFs[0].goTo(pos, 0, dT)
    timeHelper.sleep(dT)
    CFs[0].land(targetHeight=0.55, duration=0.5)
    timeHelper.sleep(3.0)
    
def button_click_exit_middle_table():
    global pos
    global CFs 
    global timeHelper
    global in_table
    if(in_table == 1):
        CFs[0].takeoff(targetHeight=0.55, duration=1)
        pos = pos + np.array([0.1, 1.15, 0.6])
        
        timeHelper.sleep(0.5)
        
        dx = pos[0] - CFs[0].initialPosition[0]
        dy = pos[1] - CFs[0].initialPosition[1]
        dist = sqrt(dx*dx+dy*dy)
        dT = dist*4

        pos[0] = CFs[0].initialPosition[0]
        pos[1] = CFs[0].initialPosition[1]

        CFs[0].goTo(pos, 0, dT)
        timeHelper.sleep(0.5)
        


    
    
def main():
    global CFs
    global timeHelper
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    CFs = allcfs.crazyflies

    global pos
    pos = np.array(CFs[0].initialPosition)

    root = Tk()
    root.title("Flight Controller")

    z_slider = Scale(root, from_=20, to=4)
    z_label = Label(root, text="Z")

    y_slider = Scale(root, from_=-10, to=15, orient=HORIZONTAL)
    y_label = Label(root, text="Y")

    x_slider = Scale(root, from_=-5, to=10, orient=HORIZONTAL)
    x_label = Label(root, text="X")

    vel_slider = Scale(root, from_=1, to=4, orient=HORIZONTAL)
    vel_label = Label(root, text="Vel")

    quit = Button(root, text="Quit", padx=120, pady=20, command=root.quit)
    quit.grid(row=0, column=0, padx=10,pady=10, columnspan=3)
    

    button_up = Button(root, text="UP", padx=20, pady=20, command=button_click_up)
    button_down = Button(root, text="DOWN", padx=20, pady=20, command=button_click_down)
    button_up_arrow = Button(root, text="⬆", padx=35, pady=20, command=lambda:button_click_up_arrow(vel_slider.get()))
    button_down_arrow = Button(root, text="⬇", padx=35, pady=20, command=lambda:button_click_down_arrow(vel_slider.get()))
    button_left_arrow = Button(root, text="⬅", padx=40, pady=20, command=lambda:button_click_left_arrow(vel_slider.get()))
    button_right_arrow = Button(root, text="➡", padx=40, pady=20, command=lambda:button_click_right_arrow(vel_slider.get()))
    button_emergency_stop = Button(root, text="Emergency Stop", padx=120, pady=20, bg="red", command=button_click_emergency)
    button_land = Button(root, text="LAND", padx=20, pady=20, command=button_click_land)
    button_takeoff=Button(root, text="TAKE OFF", padx=20, pady=20, command=button_click_takeoff)
    button_center = Button(root, text="CENTER", padx=20, pady=20, command=button_click_center)
    button_pos = Button(root, text="POS", padx=20, pady=20, command=lambda:button_click_pos(x_slider.get(),y_slider.get(),z_slider.get()))
    button_middle_table = Button(root, text="midTable", padx=20, pady=20, command=button_click_middle_table)
    button_exit_middle_table = Button(root, text="exitTable", padx=20, pady=20, command=button_click_exit_middle_table)



    button_emergency_stop.grid(row=0, column=3, columnspan=3)
    button_takeoff.grid(row=1, column=0)
    button_up.grid(row=1, column=1)
    button_center.grid(row=1, column=2)
    button_up_arrow.grid(row=1, column=3)
    button_pos.grid(row=1, column=7, pady=(0,50))
    button_land.grid(row=2, column=0)
    button_down.grid(row=2, column=1)
    button_left_arrow.grid(row=2, column=2)
    button_down_arrow.grid(row=2, column=3)
    button_right_arrow.grid(row=2, column=4)
    button_middle_table.grid(row=2, column=5)
    button_exit_middle_table.grid(row=2, column=6)

    z_slider.grid(row=1,column=6, pady=(0,30))
    z_label.grid(row=1, column=6,padx=(0, 40))

    y_slider.grid(row=0,column=7)
    y_label.grid(row=0, column=7,pady=(0, 50))

    x_slider.grid(row=1,column=7, pady=(80, 0))
    x_label.grid(row=2, column=7,pady=(0, 60))

    vel_slider.grid(row=1,column=4, padx=(40, 0))
    vel_label.grid(row=1, column=4,pady=(60, 0))


    root.mainloop()  
    
      

if __name__ == '__main__':
    main()