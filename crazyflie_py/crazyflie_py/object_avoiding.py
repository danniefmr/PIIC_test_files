import numpy as np

class SimpleProp:

    
    #positiveLimit[] = [back_limit + distance_from_back_side + back_width, left_limit + distance_from_left_side, distance_from_floor + height]
    #negativeLimit[] = [back_limit + distance_from_back_side, left_limit + distance_from_left_side + left_width, distance_from_floor]

    def __init__(self, left_width, back_width, height):
        self.positiveLimit = [back_width, 0, height]
        self.negativeLimit = [0, -left_width, 0]
        self.pos1 = [0,0,0]
        self.pos2 = [0,0,0]
        self.traj = [0,0,0]
        self.nearWall1 = [0,0,0]
        self.nearWall2 = [0,0,0]
        self.left_limit = 1.54
        self.back_limit = -0.85
        self.safe_distance = 0.18


    def coordinates(self, distance_from_left_side, distance_from_back_side, distance_from_floor, x, y, z):
        self.positiveLimit[0] += self.back_limit + distance_from_back_side + self.safe_distance
        self.positiveLimit[1] += self.left_limit - distance_from_left_side + self.safe_distance
        self.positiveLimit[2] += distance_from_floor + self.safe_distance
        
        self.negativeLimit[0] += self.back_limit + distance_from_back_side - self.safe_distance
        self.negativeLimit[1] += self.left_limit - distance_from_left_side - self.safe_distance
        self.negativeLimit[2] += distance_from_floor - self.safe_distance

        print(self.positiveLimit)
        print(self.negativeLimit)

        self.pos1 = [x,y,z]
        print("limites_positivos",self.positiveLimit)
        print("limites_negativos",self.negativeLimit)
        for i in range(3):
            if self.pos1[i] > self.positiveLimit[i]:
                self.nearWall1[i] = 1
            elif self.pos1[i] < self.negativeLimit[i]:
                self.nearWall1[i] = -1
            else:
                self.nearWall1[i] = 0
 
    def avoidProp(self, x, y, z): #retruns: true- avoid prop; False- hit prop
        self.pos2 = [x,y,z]
        for i in range(3):
            if self.pos2[i] > self.positiveLimit[i]:
                self.nearWall2[i] = 1
            elif self.pos2[i] < self.negativeLimit[i]:
                self.nearWall2[i] = -1
            else:
                self.nearWall2[i] = 0
        print("onde ele esta ",self.nearWall1)
        print("onde quer ir  ",self.nearWall2)

        if self.nearWall2[0] == 0 and self.nearWall2[1] == 0 and self.nearWall2[2] == 0:
            return False #inside prop (0 near walls)
        
        for i in range(3):
            j = i+1
            if j == 3:
                j = 0
                k = j+1
            else:
                k = j + 1
                if k == 3:
                    k = 0

            if self.nearWall1[i] == 0 and self.nearWall1[j] == 0: # 1 near wall
                if self.nearWall1[k] == self.nearWall2[k]:
                    self.savePos()
                    return True             #does not colide
                else:
                    return self.lineColision()   #could colide
                

        for i in range(3):
            j = i+1
            if j == 3:
                j = 0
                k = j+1
            else:
                k = j + 1
                if k == 3:
                    k = 0
            if self.nearWall1[i] == 0:  # 2 near walls
                if (self.nearWall1[j] == self.nearWall2[j] or self.nearWall1[k] == self.nearWall2[k]):
                    self.savePos()
                    return True
                else:
                    return self.lineColision()

        for i in range(3):
            j = i+1
            if j == 3:
                j = 0
                k = j+1
            else:
                k = j + 1
                if k == 3:
                    k = 0            
             # 3 near walls
            if (self.nearWall1[j] == self.nearWall2[j] or self.nearWall1[k] == self.nearWall2[k] or self.nearWall1[i] == self.nearWall2[i]):
                self.savePos()
                return True
            else:
                return self.lineColision()
    

    

    def lineColision(self):
        for i in range(3):
            self.traj[i] = self.pos2[i] - self.pos1[i]  #pos1: line vector, pos2: line point
        
        for i in range(3):
            j = i+1
            if j == 3:
                j = 0
                k = j+1
            else:
                k = j + 1
                if k == 3:
                    k = 0
            if self.traj[i] != 0:
                alpha = (self.positiveLimit[i]-self.pos2[i])/self.traj[i]
                y = self.pos2[j] + alpha * self.traj[j]
                z = self.pos2[k] + alpha * self.traj[k]
                if (self.negativeLimit[j] < y and y < self.positiveLimit[j] and self.negativeLimit[k] < z and self.positiveLimit[k] < z):
                    return False

        for i in range(3):
            j = i+1
            if j == 3:
                j = 0
                k = j+1
            else:
                k = j + 1
                if k == 3:
                    k = 0
            if self.traj[i] != 0:
                alpha = (self.negativeLimit[i]-self.pos2[i])/self.traj[i]
                y = self.pos2[j] + alpha * self.traj[j]
                z = self.pos2[k] + alpha * self.traj[k]
                if (self.negativeLimit[j] < y and y < self.positiveLimit[j] and self.negativeLimit[k] < z and self.positiveLimit[k] < z):
                    return False
        self.savePos()
        return True # Does not intersect none of the prop's planes
    
    def savePos(self):
        for i in range(3):
            self.pos1[i] = self.pos2[i]
            self.nearWall1[i] = self.nearWall2[i]

###########################################################


