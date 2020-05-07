import sys, random, math, pygame
from pygame.locals import *
from math import sqrt, cos, sin, atan2
import time

# constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 10.0
NUMNODES = 5000
RADIUS = 30.0
OBS = [(50, 0, 50, 300), (200, 100, 50, 400), (350, 0, 50, 300), (500, 100, 50, 400)]


class Node:
    def __init__(self, xcoord=0, ycoord=0):
        self.x = xcoord
        self.y = ycoord
        self.cost = 0
        self.parent = None


def obsDraw(pygame, screen):
    blue = (0, 0, 255)
    for o in OBS:
        pygame.draw.rect(screen, blue, o)


def dist(p1, p2):
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))


def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)


def drawPath(nodes, pygame, screen):
    last_node = nodes[-1]
    start = nodes[0]
    red = 255, 10, 10
    while last_node != start:
        pygame.draw.line(screen, red, [last_node.x, last_node.y], [last_node.parent.x, last_node.parent.y], 5)
        last_node = last_node.parent

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def checkIntersect(nodeA,nodeB,OBS):
    A=(nodeA.x,nodeA.y)
    B=(nodeB.x,nodeB.y)
    for o in OBS:
      obs=(o[0],o[1],o[0]+o[2],o[1]+o[3])
      C1=(obs[0],obs[1])
      D1=(obs[0],obs[3])
      C2=(obs[0],obs[1])
      D2=(obs[2],obs[1])
      C3=(obs[2],obs[3])
      D3=(obs[2],obs[1])
      C4=(obs[2],obs[3])
      D4=(obs[0],obs[3])
      inst1= ccw(A,C1,D1) != ccw(B,C1,D1) and ccw(A,B,C1) != ccw(A,B,D1)
      inst2= ccw(A,C2,D2) != ccw(B,C2,D2) and ccw(A,B,C2) != ccw(A,B,D2)
      inst3= ccw(A,C3,D3) != ccw(B,C3,D3) and ccw(A,B,C3) != ccw(A,B,D3)
      inst4= ccw(A,C4,D4) != ccw(B,C4,D4) and ccw(A,B,C4) != ccw(A,B,D4)
      if inst1==False and inst2==False and inst3==False and inst4==False:
        #print(A,B)
        #input("Press Enter to continue...")
        continue
      else:
         return False
    return True

def checkgoal(newnode, goal):
    if dist([goal.x, goal.y], [newnode.x, newnode.y]) < RADIUS and checkIntersect(newnode, goal, OBS):
        return True


def main():
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRT_star')
    white = 255, 255, 255
    black = 20, 20, 40
    screen.fill(white)
    obsDraw(pygame, screen)

    nodes = []

    start = Node(0.0, 0.0)  # Start in the corner
    goal = Node(630.0, 470.0)

    nodes.append(start)
    i = 0
    start_time = time.time()
    while i < NUMNODES and goal not in nodes:

        rand = Node(random.random() * XDIM, random.random() * YDIM)

        nn = nodes[0]
        for p in nodes:
            if dist([p.x, p.y], [rand.x, rand.y]) < dist([nn.x, nn.y], [rand.x, rand.y]):
                nn = p

        interpolatedNode = step_from_to([nn.x, nn.y], [rand.x, rand.y])
        newnode = Node(interpolatedNode[0], interpolatedNode[1])

        if checkIntersect(nn, newnode, OBS):
            newnode.cost = nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y])
            newnode.parent = nn
            nodes.append(newnode)
            pygame.draw.line(screen, black, [nn.x, nn.y], [newnode.x, newnode.y])

        if checkgoal(newnode, goal):
            print("Path found")
            goal.cost = newnode.cost + dist([newnode.x, newnode.y], [goal.x, goal.y])
            goal.parent = newnode
            nodes.append(goal)
            pygame.draw.line(screen, black, [newnode.x, newnode.y], [goal.x, goal.y])
        pygame.display.update()
        i += 1

        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")

    if goal in nodes:
        end_time = time.time()
        time_taken = end_time - start_time
        total_cost = nodes[-1].cost
        print("")
        print("RRT* Stats")
        print("")
        print("Cost       : " + str(total_cost) + ' units')
        print("Time Taken : " + str(time_taken) + ' sec')

        drawPath(nodes, pygame, screen)
        pygame.display.update()
    # pygame.image.save(screen, "rrt.jpg")
    else:
        print("Path not found. Try increasing the number of iterations")


if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False



