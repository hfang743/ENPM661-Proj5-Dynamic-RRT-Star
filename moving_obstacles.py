# ENPM661
# Project 5
# Jaad Lepak
# ===== ===== =====

# ===== Libraries =====
import math, sys, pygame, random
from math import *
from pygame import *

# ===== Initialization =====
class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent

XDIM = 720
YDIM = 500
windowSize = [XDIM, YDIM]
delta = 10.0
GAME_LEVEL = 1
GOAL_RADIUS = 10
MIN_DISTANCE_TO_ADD = 1.0
NUMNODES = 5000
pygame.init()
fpsClock = pygame.time.Clock()
screen = pygame.display.set_mode(windowSize)
white = 255, 255, 255
black = 0, 0, 0
red = 255, 0, 0
blue = 0, 255, 0
green = 0, 0, 255
cyan = 0,180,105

count = 0
rectObs = []

def dist(p1,p2):    #distance between two points
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def point_circle_collision(p1, p2, radius):
    distance = dist(p1,p2)
    if (distance <= radius):
        return True
    return False

def step_from_to(p1,p2):
    if dist(p1,p2) < delta:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + delta*cos(theta), p1[1] + delta*sin(theta)

def collides(p):    #check if point collides with the obstacle
    for rect in rectObs:
        if rect.collidepoint(p) == True:
            return True
    return False


def get_random_clear():
    while True:
        p = random.random()*XDIM, random.random()*YDIM
        noCollision = collides(p)
        if noCollision == False:
            return p

# ===== Obstacles =====
def draw_rect_obs(list):
    """
    Setup for rectangle obstacle

    Inputs:
        startX:     Left-corner X position
        startY:     Left-corner Y position
        lengthX:    Length of rectangle
        lengthY:    Width of rectangle
        speedX:     Units per tick in X direction
        speedY:     Units per tick in Y direction
    """
    # Input list
    startX = list[0]
    startY = list[1]
    lengthX = list[2]
    lengthY = list[3]

    # Draw Obstacle
    print(list)
    pygame.draw.rect(screen, black, \
    pygame.Rect((startX, startY),(lengthX, lengthY)))

def dir(list):
    """
    Returns an array to change moving obstacle direction once edge of map is reached

    Inputs:
        startX:     Left-corner X position
        startY:     Left-corner Y position
        lengthX:    Length of rectangle
        lengthY:    Width of rectangle
        speedX:     Units per tick in X direction
        speedY:     Units per tick in Y direction

    Outputs:
        [1,1]:      Left or top side reached map edge
        [-1,-1]:    Right or bottom side reached map edge
        [1,-1]:     Left or bottom side reached map edge
        [-1,1]:     Right or top side reached map edge
    """
    # Input list
    startX = list[0]
    startY = list[1]
    lengthX = list[2]
    lengthY = list[3]
    speedX = list[4]
    speedY = list[5]
    dirX = list[6]
    dirY = list[7]

    # Check if obstacle reached edge of map
    if (startX + speedX) < 0:
        dirX = 1
    if (startX + lengthX + speedX) > XDIM:
        dirX = -1
    if (startY + speedY) < 0:
        dirY = 1
    if (startY + lengthY + speedY) > YDIM:
        dirY = -1

    return [dirX,dirY]

def move(rectObs):
    """
    Moves obstacles and returns updated positions
    """
    updated_rectObs = []
    screen.fill(white)
    for list in rectObs:
        # Input list
        [dirX,dirY] = dir(list)
        startX = list[0]
        startY = list[1]
        lengthX = list[2]
        lengthY = list[3]
        speedX = list[4]
        speedY = list[5]

        startX += speedX*dirX
        startY += speedY*dirY

        rect = [startX,startY,lengthX,lengthY]
        draw_rect_obs(rect)
        updated_rectObs.append([startX,startY,lengthX,lengthY,speedX,speedY,dirX,dirY])

    return updated_rectObs

def reset():
    global count
    screen.fill(white)
    count = 0

# ===== Program =====
def main():
    global count

    initPoseSet = False
    initialPoint = Node(None, None)
    goalPoseSet = False
    goalPoint = Node(None, None)
    currentState = 'init'

    # Obstacles
    rectObs = []
    """
    For each rectangle obstacle, put in an EIGHT (8) item list with the following:
        Start X position (0 starts on left side)
        Start Y position (0 starts on top side)
        X dimensions of obstacle
        Y dimensions of obstacle
        X units to move per tick
        Y units to move per tick
        X starting direction (1 for right, -1 for left)
        Y starting direction (1 for down, -1 for up)

    Obstacles will bounce off boder and stay within map
    """
    # Rectangle Obstacle 1
    rectObs.append([50,50,100,100,0,10,1,1])
    # Rectangle Obstacle 2
    rectObs.append([400,200,100,100,0,10,1,1])

    nodes = []
    reset()

    while True:
        if currentState == 'init':
            print('goal point not yet set')
            pygame.display.set_caption('Select Starting Point and then Goal Point')
            fpsClock.tick(10)
        elif currentState == 'goalFound':
            currNode = goalNode.parent
            pygame.display.set_caption('Goal Reached')
            print ("Goal Reached")


            while currNode.parent != None:
                pygame.draw.line(screen,red,currNode.point,currNode.parent.point)
                currNode = currNode.parent
            optimizePhase = True
        elif currentState == 'optimize':
            fpsClock.tick(0.5)
            pass
        elif currentState == 'buildTree':
            count = count+1
            pygame.display.set_caption('Performing RRT')
            if count < NUMNODES:
                foundNext = False
                while foundNext == False:
                    rand = get_random_clear()
                    parentNode = nodes[0]
                    for p in nodes:
                        if dist(p.point,rand) <= dist(parentNode.point,rand):
                            newPoint = step_from_to(p.point,rand)
                            if collides(newPoint) == False:
                                parentNode = p
                                foundNext = True

                newnode = step_from_to(parentNode.point,rand)
                nodes.append(Node(newnode, parentNode))
                pygame.draw.line(screen,cyan,parentNode.point,newnode)

                if point_circle_collision(newnode, goalPoint.point, GOAL_RADIUS):
                    currentState = 'goalFound'

                    goalNode = nodes[len(nodes)-1]


            else:
                print("Ran out of nodes... :(")
                return;

        #handle events
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                print('mouse down')
                if currentState == 'init':
                    if initPoseSet == False:
                        nodes = []
                        if collides(e.pos) == False:
                            print('initiale point set: '+str(e.pos))

                            initialPoint = Node(e.pos, None)
                            nodes.append(initialPoint) # Start in the center
                            initPoseSet = True
                            pygame.draw.circle(screen, red, initialPoint.point, GOAL_RADIUS)
                    elif goalPoseSet == False:
                        print('goal point set: '+str(e.pos))
                        if collides(e.pos) == False:
                            goalPoint = Node(e.pos,None)
                            goalPoseSet = True
                            pygame.draw.circle(screen, green, goalPoint.point, GOAL_RADIUS)
                            currentState = 'buildTree'
                else:
                    currentState = 'init'
                    initPoseSet = False
                    goalPoseSet = False
                    reset()

        rectObs = move(rectObs)

        pygame.display.update()
        fpsClock.tick(10000)



if __name__ == '__main__':
    main()
