"""
INCOMPLETE
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
OBS=[(200,300,200,100), (350,100,100,100)]



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

def showRobot(node, pygame, screen):
    x = int(node.x)
    y = int(node.y)
    pygame.draw.circle(screen, green, [x,y], 5)
    pygame.display.update()

def animateRobot(nodes, pygame, screen):
    last_node = nodes[-1]
    start = nodes[0]
    global green
    green = 0, 255, 0
    while last_node != start:
        showRobot(last_node, pygame, screen)
        last_node = last_node.parent


def main():
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('Bi-directional_RRT_star')
    white = 255, 255, 255
    black = 20, 20, 40
    green = 0, 255, 0
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
        animateRobot(start_nodes, pygame, screen)
        animateRobot(goal_nodes, pygame, screen)
        pygame.display.update()

        # pygame.image.save(screen, "bi_rrt_extend_both.jpg")
    else:
        print("Path not found. Try increasing the number of iterations")


if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False