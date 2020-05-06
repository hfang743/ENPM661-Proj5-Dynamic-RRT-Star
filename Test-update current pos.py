import pygame,random
import math
import time
import numpy,sys
from pygame.locals import *
from numpy import *
import matplotlib.path as mplPath
Xmax = 640
Ymax = 480
esp = 20.00
n_of_nodes = 1000
obstacle_1 = []
obstacle_2 = []
x_pos=100
y_pos=200
x1_pos=50
y1_pos=80
vel=30
vel1=20
#obstacle
for i in range(20):
    x = x_pos + 20 * math.cos(i * 2 * math.pi / 20.0)
    y = y_pos + 20 * math.sin(i * 2 * math.pi / 20.0)
    obstacle_1.append([x,y])

# for i in range(20):
#     x1 = x1_pos + 20 * math.cos(i * 2 * math.pi / 20.0)
#     y1 = y1_pos + 20 * math.sin(i * 2 * math.pi / 20.0)
#     obstacle_2.append([x1,y1])
class Node:
    x = 0
    y = 0
    cost = 0
    parent = None
    def __init__(self, xcord, ycord):
        self.x = xcord
        self.y = ycord

def dist(x1 ,y1, x2,y2):
    D = math.sqrt(math.pow(x2-x1,2)+ math.pow(y2-y1,2))
    return D


def steer(qrand, qnear,ndist,node,screen,black):
    qnew = Node(0,0)
    if min(ndist) >= esp:
        qnew.x = qnear.x + ((qrand.x - qnear.x) * esp) / dist(qrand.x, qrand.y, qnear.x, qnear.y)
        qnew.y = qnear.y + ((qrand.y - qnear.y) * esp) / dist(qrand.x, qrand.y, qnear.x, qnear.y)
    else:
        qnew.x = qrand.x
        qnew.y = qrand.y

    t1 = bbPath1.contains_point((qnew.x, qnew.y))
    if t1 == 1:
        return qnear
    # t2 = bbPath2.contains_point((qnew.x, qnew.y))
    # if t2 == 1:
    #     return qnear

    qnew.parent = node.index(qnear)

    # pygame.draw.aaline(screen, black, [qnear.x, qnear.y], [qnew.x, qnew.y])
    # pygame.display.update()
    return qnew


def main():
    #node = []
    #qinit = Node(0,0)
    qinit.cost = 0
    qinit.parent = None
    node.append(qinit)

    for v in range(1,n_of_nodes):
        p=random.random()
        if p<0.5:
            qxrand = random.randint(0,640)
            qyrand = random.randint(0,480)
        else:
            qxrand = qgoal.x
            qyrand = qgoal.y
        if [qgoal.x, qgoal.y] in node:
            print("destination reached")
            break

        qrand = Node(qxrand,qyrand)
        ndist = []
        for n in node:
            tmp = dist(n.x, n.y, qrand.x, qrand.y)
            ndist.append(tmp)
        qnear = node[ndist.index(min(ndist))]
        qnew = steer(qrand, qnear,ndist, node,screen,black)
        qnew.cost = dist(qnew.x,qnew.y, qnear.x,qnear.y) + qnear.cost
        #qnew.parent =node.index(qnear)
        node.append(qnew)

    D = []
    for n in node:
        tmpdist = dist(n.x, n.y, qgoal.x, qgoal.y)
        D.append(tmpdist)

    qfinal = node[D.index(min(D))]
    node.append(qgoal)

    qgoal.parent = node.index(qfinal)




    end = qgoal
    sta = qfinal

    while end.parent is not None:
        sta = int(end.parent)
        path.append(node[sta])
        pygame.draw.aaline(screen, Red, [end.x, end.y], [node[sta].x, node[sta].y])
        pygame.display.update()
        end = node[sta]

if __name__ == '__main__':

    pygame.init()
    screen = pygame.display.set_mode([640, 480])
    pygame.display.set_caption('dynamic_rrt')
    Green = 255,255,255
    black = 0, 0, 0
    Red = 255, 0 ,0
    screen.fill(Green)
    running = True
    qinit = Node(0, 0)

    while running:
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        while True :
            screen.fill(Green)
            if x_pos> 500 or x_pos< 50 :
                vel = -vel
            if vel>0:
                temp1vel = 1
            if vel<0:
                temp1vel = -1

            if dist(qinit.x,qinit.y,x_pos,y_pos)<100:
                vel=0
            else:
                vel=temp1vel*30

            if x1_pos> 500 or x1_pos< 50 :
                vel1 = -vel1
            if vel1>0:
                temp2vel = 1
            if vel1<0:
                temp2vel = -1
            if dist(qinit.x, qinit.y, x1_pos, y1_pos) < 100:
                vel1 = 0
            else:
                vel1 = temp2vel*20



            qgoal = Node(400, 400)
            bbPath1 = mplPath.Path(array(obstacle_1))
            pygame.draw.polygon(screen, Red, obstacle_1, 0)
            # bbPath2 = mplPath.Path(array(obstacle_2))
            # pygame.draw.polygon(screen, Red, obstacle_2, 0)
            #pygame.display.update()
            node = []
            path = []

            if dist(qgoal.x,qgoal.y,qinit.x,qinit.y)<20:
                break
            pygame.draw.circle(screen, (254, 121, 209), [qinit.x, qinit.y], 20, 0)
            pygame.draw.circle(screen, (0, 255, 0), [qgoal.x, qgoal.y], 20, 0)
            main()
            time.sleep(.05)
            carvel=300
            qtime1=Node(path[-1].x,path[-1].y)
            qtime2=Node(path[-2].x,path[-2].y)
            distance=dist(qtime1.x,qtime1.y,qtime2.x,qtime2.y)
            theta1=math.atan2((qtime2.y-qtime1.y),(qtime2.x-qtime1.x))
            tempx=qtime1.x+carvel*0.05*math.cos(theta1)
            tempy=qtime1.y+carvel*0.05*math.sin(theta1)
            if distance>carvel*0.05:
                # if (dist(qinit.x,qinit.y,x_pos,y_pos)>100)  and (dist(qinit.x,qinit.y,x1_pos,y1_pos)>100):
                    qinit=Node(int(tempx),int(tempy))

            obstacle_1=[]
            # obstacle_2=[]
            x_pos += vel
            x1_pos+= vel1
            for i in range(25):
                x = x_pos + 20 * math.cos(i * 2 * math.pi / 20.0)
                y = y_pos + 20 * math.sin(i * 2 * math.pi / 20.0)
                obstacle_1.append([x, y])

            # for i in range(20):
            #     x1 = x1_pos + 20 * math.cos(i * 2 * math.pi / 20.0)
            #     y1 = y1_pos + 20 * math.sin(i * 2 * math.pi / 20.0)
            #     obstacle_2.append([x1, y1])
