"""
INCOMEPLETE
"""

"""
implementation of paper by Sertac Karaman in 2010
http://roboticsproceedings.org/rss06/p34.pdf
"""
import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
import time
#constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 10.0
NUMNODES = 5000 #samples/iterations
RADIUS = 30.0
TARGET_RADIUS = 200.0

# ===== Dynamic Obstacles =====
"""
For each rectangle obstacle, list with the following:
    Start X position (0 starts on left side)
    Start Y position (0 starts on top side)
    X dimensions of obstacle
    Y dimensions of obstacle
"""
# Obstacles
global OBS
OBS = [[50,50,100,100]]
OBS.append([400,200,100,100])
OBS.append([200,100,100,100])
"""
For each rectangle obstacle, list with the following:
    X units to move per tick
    Y units to move per tick
    X starting direction (1 for right, -1 for left)
    Y starting direction (1 for down, -1 for up)

Obstacles will bounce off boder and stay within map
"""
# Motion informatoin
OBS_motion = [[0,0.1,1,1]]
OBS_motion.append([0,0.1,1,1])
OBS_motion.append([0.1,0,1,1])

def dir(obs,obs_motion):
    """
    Returns an array to change moving obstacle direction once edge of map is reached

    Inputs:
        startX:     Left-corner X position
        startY:     Left-corner Y position
        lengthX:    Length of rectangle
        lengthY:    Width of rectangle

        moveX:     Units per tick in X direction
        moveY:     Units per tick in Y direction
        dirX:      1 for left, -1 for right
        dirY:      1 for down, -1 for up
    Outputs:
        [1,1]:      Left or top side reached map edge
        [-1,-1]:    Right or bottom side reached map edge
        [1,-1]:     Left or bottom side reached map edge
        [-1,1]:     Right or top side reached map edge
    """
    # Input list
    startX = obs[0]
    startY = obs[1]
    lengthX = obs[2]
    lengthY = obs[3]

    moveX = obs_motion[0]
    moveY = obs_motion[1]
    dirX = obs_motion[2]
    dirY = obs_motion[3]

    # Check if obstacle reached edge of map
    if (startX + moveX) < 0:
        dirX = 1
    if (startX + lengthX + moveX) > XDIM:
        dirX = -1
    if (startY + moveY) < 0:
        dirY = 1
    if (startY + lengthY + moveY) > YDIM:
        dirY = -1

    return [dirX,dirY]

def move(obs,obs_motion):
    """
    Moves obstacles and returns updated positions
    """
    # Input list
    startX = obs[0]
    startY = obs[1]
    lengthX = obs[2]
    lengthY = obs[3]

    moveX = obs_motion[0]
    moveY = obs_motion[1]

    # Change direction if obstacle reached end of map
    [dirX,dirY] = dir(obs,obs_motion)

    startX += moveX*dirX
    startY += moveY*dirY

    # Output
    obs[0] = startX
    obs[1] = startY
    obs_motion[2] = dirX
    obs_motion[3] = dirY

    return obs,obs_motion

def updateObs():
    """
    Update moving obstacle location on map
    """
    global screen
    global white
    # Count number of obstacles
    numObs = 0
    for o in OBS:
        numObs += 1 # number of obstacles
    # Update moving obstacle position
    for i in range(0, numObs):
        OBS[i], OBS_motion[i] = move(OBS[i],OBS_motion[i])


class Node:
    def __init__(self, xcoord=0, ycoord=0, cost=0, parent=None):
        self.x = xcoord
        self.y = ycoord
        self.cost = cost
        self.parent = parent


def obsDraw(pygame, screen):
    blue = (0, 0, 255)
    for o in OBS:
        pygame.draw.rect(screen, blue, o)


def dist(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)


def chooseParent(nn, newnode, nodes):
    for p in nodes:
        if checkIntersect(p, newnode, OBS) and dist([p.x, p.y], [newnode.x, newnode.y]) < RADIUS and p.cost + dist(
                [p.x, p.y], [newnode.x, newnode.y]) < nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y]):
            nn = p
        newnode.cost = nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y])
        newnode.parent = nn
    return newnode, nn


def nearest_neighbor(nodes, q_target):
    q_near = nodes[0]
    for p in nodes:
        if dist([p.x, p.y], [q_target.x, q_target.y]) < dist([q_near.x, q_near.y], [q_target.x, q_target.y]):
            q_near = p
    return q_near


def ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


# Return true if line segments AB and CD intersect
def checkIntersect(nodeA, nodeB, OBS):
    A = (nodeA.x, nodeA.y)
    B = (nodeB.x, nodeB.y)
    for o in OBS:
        obs = (o[0], o[1], o[0] + o[2], o[1] + o[3])
        C1 = (obs[0], obs[1])
        D1 = (obs[0], obs[3])
        C2 = (obs[0], obs[1])
        D2 = (obs[2], obs[1])
        C3 = (obs[2], obs[3])
        D3 = (obs[2], obs[1])
        C4 = (obs[2], obs[3])
        D4 = (obs[0], obs[3])
        inst1 = ccw(A, C1, D1) != ccw(B, C1, D1) and ccw(A, B, C1) != ccw(A, B, D1)
        inst2 = ccw(A, C2, D2) != ccw(B, C2, D2) and ccw(A, B, C2) != ccw(A, B, D2)
        inst3 = ccw(A, C3, D3) != ccw(B, C3, D3) and ccw(A, B, C3) != ccw(A, B, D3)
        inst4 = ccw(A, C4, D4) != ccw(B, C4, D4) and ccw(A, B, C4) != ccw(A, B, D4)
        if inst1 == False and inst2 == False and inst3 == False and inst4 == False:
            # print(A,B)
            # input("Press Enter to continue...")
            continue
        else:
            return False
    return True


def extend(nodes, screen, black):
    rand = Node(random.random() * XDIM, random.random() * YDIM)
    nn = nearest_neighbor(nodes, rand)
    interpolatedNode = step_from_to([nn.x, nn.y], [rand.x, rand.y])
    newnode = Node(interpolatedNode[0], interpolatedNode[1])

    if checkIntersect(nn, newnode, OBS):
        [newnode, nn] = chooseParent(nn, newnode, nodes)
        nodes.append(newnode)
        pygame.draw.line(screen, black, [nn.x, nn.y], [newnode.x, newnode.y])
        pygame.display.update()

    for e in pygame.event.get():
        if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
            sys.exit("Leaving because you requested it.")

    return nodes


def drawPath(nodes, pygame, screen):
    last_node = nodes[-1]
    start = nodes[0]
    red = 255, 10, 10
    while last_node != start:
        pygame.draw.line(screen, red, [last_node.x, last_node.y], [last_node.parent.x, last_node.parent.y], 5)
        last_node = last_node.parent

def checkCollision(nodes, OBS):
    """
    Checks if the optimal path is being blocked by an obstacle
    """
    last_node = nodes[-2]
    start = nodes[0]
    while last_node != start:
        if checkIntersect(last_node, last_node.parent, OBS) == False:
            return True

    return False

def refresh():
    pygame.init()
    global screen
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('Bi-directional_RRT_star')
    global white
    white = 255, 255, 255
    black = 20, 20, 40
    screen.fill(white)
    obsDraw(pygame, screen)
    pygame.display.update()


def main():
    pygame.init()
    global screen
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('Bi-directional_RRT_star')
    global white
    white = 255, 255, 255
    black = 20, 20, 40
    screen.fill(white)
    obsDraw(pygame, screen)

    start = Node(0.0, 0.0)  # Start in the corner
    goal = Node(630.0, 200.0)

    start_nodes = []
    goal_nodes = []

    start_nodes.append(start)
    goal_nodes.append(goal)

    flag = False
    i = 0
    start_time = time.time()
    while i < NUMNODES and flag != True:

        start_nodes = extend(start_nodes, screen, black)


        goal_nodes = extend(goal_nodes, screen, black)
        q_target = goal_nodes[-1]

        # try to connect q_near and q_target if dist is less than a target radius
        q_near = nearest_neighbor(start_nodes, q_target)
        if (dist([q_target.x, q_target.y], [q_near.x, q_near.y]) < TARGET_RADIUS):
            if checkIntersect(q_near, q_target, OBS):
                newnode = Node(q_target.x, q_target.y)
                [newnode, nn] = chooseParent(q_near, newnode, start_nodes)
                start_nodes.append(newnode)
                pygame.draw.line(screen, black, [q_near.x, q_near.y], [newnode.x, newnode.y])
                flag = True
                print("Path found")
                break

        i += 1
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")


    if flag == True:
        end_time = time.time()
        time_taken = end_time - start_time
        total_cost = start_nodes[-1].cost + goal_nodes[-1].cost

        print("Cost       : " + str(total_cost) + ' units')
        print("Time Taken : " + str(time_taken) + ' s')

        drawPath(start_nodes, pygame, screen)
        drawPath(goal_nodes, pygame, screen)

        global START_NODES
        global GOAL_NODES
        START_NODES = start_nodes
        GOAL_NODES = goal_nodes

        pygame.display.update()
        #time.sleep(0.01)
        # pygame.image.save(screen, "bi_rrt_extend_both.jpg")
    else:
        print("Path not found. Try increasing the number of iterations")


if __name__ == '__main__':
    main()
    running = True
    while running:
        updateObs()
        refresh()

        # EXITS program using ESC
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
