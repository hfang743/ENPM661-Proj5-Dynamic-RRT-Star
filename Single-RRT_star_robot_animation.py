# ENPM661
# Project 5
# Jaad Lepak
# Haixiang Fang
# Anshuman Singh
# ===== ===== =====

# ===== Libraries =====
import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
import time

# ===== Constants =====
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 10.0
NUMNODES = 5000 #samples/iterations
RADIUS = 30.0
TARGET_RADIUS = 10.0

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
OBS = [[50,50,50,50]]
OBS.append([150,150,50,50])
OBS.append([250,250,50,50])
OBS.append([350,350,50,50])
OBS.append([450,250,50,50])
OBS.append([550,150,50,50])

"""
For each rectangle obstacle, list with the following:
    X units to move per tickhttps://stackoverflow.com/questions/16994243/pygame-not-filled-shapes-python
    Y units to move per tick
    X starting direction (1 for right, -1 for left)
    Y starting direction (1 for down, -1 for up)

Obstacles will bounce off boder and stay within map
"""
# Motion informatoin
OBS_motion = [[0,3,1,1]]
OBS_motion.append([0,2,1,-1])
OBS_motion.append([0,2,1,-1])
OBS_motion.append([0,2,1,1])
OBS_motion.append([0,2,1,1])
OBS_motion.append([0,2,1,1])


def dir(obs,obs_motion,OBS):
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

    # Check if obstacle reaches edge of another obstacle
    gap = 2
    for o in OBS:
        if o != obs:
            # Check top-left corner
            if (startX + moveX) > o[0] and (startY + moveY) > o[1]\
            and (startX + moveX) < (o[0] + o[2]) and (startY + moveY) < (o[1] + o[3]):
                dirX *= -1
                dirY *= -1
            # Check top-right corner
            if (startX + moveX + lengthX) > o[0] and (startY + moveY) > o[1]\
            and (startX + moveX + lengthX) < (o[0] + o[2]) and (startY + moveY) < (o[1] + o[3]):
                dirX *= -1
                dirY *= -1
            # Check bottom-left corner
            if (startX + moveX) > o[0] and (startY + moveY + lengthY) > o[1]\
            and (startX + moveX) < (o[0] + o[2]) and (startY + moveY + lengthY) < (o[1] + o[3]):
                dirX *= -1
                dirY *= -1
            # Check bottom-right corner
            if (startX + moveX + lengthX) > o[0] and (startY + moveY + lengthY) > o[1]\
            and (startX + moveX + lengthX) < (o[0] + o[2]) and (startY + moveY + lengthY) < (o[1] + o[3]):
                dirX *= -1
                dirY *= -1

    return [dirX,dirY]

def move(obs,obs_motion,OBS):
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
    [dirX,dirY] = dir(obs,obs_motion,OBS)

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
    # Count number of obstacles
    numObs = 0
    for o in OBS:
        numObs += 1 # number of obstacles
    # Update moving obstacle position
    for i in range(0, numObs):
        OBS[i], OBS_motion[i] = move(OBS[i],OBS_motion[i],OBS)


# ===== Node class =====
class Node:
    def __init__(self, xcoord=0, ycoord=0, cost=0, parent=None):
        self.x = xcoord
        self.y = ycoord
        self.cost = cost
        self.parent = parent

# ===== Functions =====
def obsDraw(pygame, screen):
    """
    Draws all obstacles
    """
    blue = (0, 0, 255)
    for o in OBS:
        pygame.draw.rect(screen, blue, o)

    pygame.display.update()


def dist(p1, p2):
    """
    Calculates the distance between two points
    """
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def step_from_to(p1, p2):
    """
    Used to interpolate between two points
    """
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)


def chooseParent(nn, newnode, nodes):
    """
    Chooses a parent node for a newly found node
    """
    for p in nodes:
        if checkIntersect(p, newnode, OBS) and dist([p.x, p.y], [newnode.x, newnode.y]) < RADIUS and p.cost + dist(
                [p.x, p.y], [newnode.x, newnode.y]) < nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y]):
            nn = p
        newnode.cost = nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y])
        newnode.parent = nn
    return newnode, nn


def nearest_neighbor(nodes, q_target):
    """
    Connects the nodes in the branch
    """
    q_near = nodes[0]
    for p in nodes:
        if dist([p.x, p.y], [q_target.x, q_target.y]) < dist([q_near.x, q_near.y], [q_target.x, q_target.y]):
            q_near = p
    return q_near


def ccw(A, B, C):
    """
    A, B, C are three x,y coordinate positions
    """
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


# Return true if line segments AB and CD intersect
def checkIntersect(nodeA, nodeB, OBS):
    """
    Returns true if AB and CD intersect with any Obstacles
    AB is a line drawn from one point to another
    A and B are given as nodes
    CD represents each side of each obstacle
    """
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
    """
    Extends random nodes used in RRT search
    """
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
    """
    Draws the optimal path
    """
    last_node = nodes[-1]
    start = nodes[0]
    while last_node != start:
        pygame.draw.line(screen, red, [last_node.x, last_node.y], [last_node.parent.x, last_node.parent.y], 5)
        last_node = last_node.parent

def checkCollision(robot, OBS):
    """
    Checks if robot sensor range sees an obstacle
    Returns true if square range around robot intersects with an obstacle
    """
    # Import robot parameters
    startX = robot[0]
    startY = robot[1]
    length = robot[2]
    width = robot[3]

    # Check if robot range reaches edge of another obstacle
    for o in OBS:
        # Check top-left corner
        if (startX - length) >= o[0] and (startY - width) >= o[1]\
        and (startX - length) <= (o[0] + o[2]) and (startY - width) <= (o[1] + o[3]):
            return True
        # Check top-right corner
        if (startX + length) >= o[0] and (startY - width) >= o[1]\
        and (startX + length) <= (o[0] + o[2]) and (startY - width) <= (o[1] + o[3]):
            return True
        # Check bottom-left corner
        if (startX - length) >= o[0] and (startY + width) >= o[1]\
        and (startX - length) <= (o[0] + o[2]) and (startY + width) <= (o[1] + o[3]):
            return True
        # Check bottom-right corner
        if (startX + length) >= o[0] and (startY + width) >= o[1]\
        and (startX + length) <= (o[0] + o[2]) and (startY + width) <= (o[1] + o[3]):
            return True

    return False

def pathDraw(startNodes,goalNodes):
    """
    Draws red line for optimal path using nodes from start and nodes from goal
    """
    drawPath(startNodes, pygame, screen)
    drawPath(goalNodes, pygame, screen)
    pygame.display.update()

def checkgoal(newnode, goal):
    if dist([goal.x, goal.y], [newnode.x, newnode.y]) < RADIUS and checkIntersect(newnode, goal, OBS):
        return True

def RRT_Star(start,goal):
    """
    Runs Single RRT* search and returns optimal path
    """
    nodes = []

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
            [newnode, nn] = chooseParent(nn, newnode, nodes)
            nodes.append(newnode)
            pygame.draw.line(screen, black, [nn.x, nn.y], [newnode.x, newnode.y])

        if checkgoal(newnode, goal):
            print("Path found")
            [goal, newnode] = chooseParent(newnode, goal, nodes)
            nodes.append(goal)

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
    # pygame.image.save(screen, "rrt_star.jpg")
    else:
        print("Path not found. Try increasing the number of iterations")

    totalNodes = i

    return nodes, totalNodes


def recordPath(nodes):
    """
    Captures all nodes from start to goal
    """
    path = []

    # Extract nodes
    last_node = nodes[-1]
    start = nodes[0]
    while last_node != start:
        path.insert(0,last_node)
        last_node = last_node.parent

    return path


def animateRobot(x,y):
    """
    Shows a black circle as the robot moving along optimal path
    """
    pygame.draw.ellipse(screen, black, [x - 5,y - 5,10,10])
    pygame.display.update()

def animateSensor(x,y,length,width):
    """
    Shows a green square as the robot's vision moving along optimal path
    """
    pygame.draw.rect(screen, green, [x - length, y - width, length*2, width*2], 2 )
    pygame.display.update()


def startGoalDraw(start,goal):
    """
    Draw green circle for start
    Draw red circle for goal
    """
    pygame.draw.ellipse(screen, green, [start.x - 10,start.y - 10,20,20])
    pygame.draw.ellipse(screen, red, [goal.x - 10,goal.y - 10,20,20])
    pygame.display.update()

def allowExit():
    """Allows user to press ESC to exit"""
    for e in pygame.event.get():
        if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
            sys.exit("Leaving because you requested it.")

# ===== Main Program =====
def main():
    # Initialization
    pygame.init()
    global screen
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('Bi-directional_RRT_star')
    global white
    global black
    global green
    global red
    white = 255, 255, 255
    black = 20, 20, 40
    green = 0, 255, 0
    red = 255, 10, 10
    screen.fill(white)
    obsDraw(pygame, screen)

    # Start and goal positions
    start = Node(5.0, 5.0)  # Start in top-left corner
    goal = Node(525, 350.0) # End in bottom-right corner

    # Record start time
    startTime = time.time()
    totalNodes = 0
    # Do first search
    PATH_NODES, numNodes = RRT_Star(start,goal)
    totalNodes += numNodes
    optimalPath = recordPath(PATH_NODES)

    # Start tracking
    goal_reached = False
    counter = 0
    totalCost = 0

    # Begin robot animation
    while goal_reached == False:

        pos = optimalPath[counter]

        # Check if robot reached goal
        if pos == optimalPath[-1]:
            screen.fill(white)
            updateObs() # Move stored position of obstacles
            obsDraw(pygame, screen) # Draw obstacles on map
            drawPath(PATH_NODES, pygame, screen) # Draw last known optimal path
            startGoalDraw(start,goal) # Draw start and goal positions
            animateRobot(goal.x,goal.y) # Shows robot as green circle along optimal path
            goal_reached = True
            endTime = time.time()
            totalTime = endTime - startTime
            print("")
            print("===== GOAL REACHED =====")
            print("Total time: " + str(totalTime) + " seconds")
            print("Total cost: " + str(totalCost))
            print("Total nodes explored: " + str(totalNodes))
            print("Close window to exit.")
            break


        # Move robot from current node to next node
        robotStart = [pos.x,pos.y] # Robot at current node
        robotEnd = [optimalPath[counter + 1].x, optimalPath[counter + 1].y] # Robot traveling to next node

        # Increment counter for next node
        counter += 1

        # Define an integer for robot speed (1 is fastest, higher numbers slow down robot)
        robotSpeed = 10
        for inc in range(1,robotSpeed):
            screen.fill(white)
            updateObs() # Move stored position of obstacles
            obsDraw(pygame, screen) # Draw obstacles on map
            drawPath(PATH_NODES, pygame, screen) # Draw last known optimal path
            startGoalDraw(start,goal) # Draw start and goal positions
            xPos = robotStart[0] + (robotEnd[0] - robotStart[0])*(inc/(robotSpeed*1.0))
            yPos = robotStart[1] + (robotEnd[1] - robotStart[1])*(inc/(robotSpeed*1.0))
            animateRobot(xPos,yPos) # Shows robot as green circle along optimal path
            totalCost += (robotEnd[0] - robotStart[0])*(inc/(robotSpeed*1.0))

            # Track path in front of robot
            # Sensor 1
            l1 = 20
            w1 = 20
            sensor1 = [xPos,yPos,l1,w1]
            animateSensor(xPos,yPos,l1,w1)
            # If any obstacle blocks path around robot
            if checkCollision(sensor1, OBS) == True:
                print("OBSTACLE DETECTED. EXPLORING NEW PATH TO GOAL.")
                start = Node(xPos,yPos) # New start position is robot's current position
                PATH_NODES, numNodes = RRT_Star(start,goal)
                totalNodes += numNodes
                optimalPath = recordPath(PATH_NODES)
                counter = 0 # Reset counter of nodes for optimal path
                pos = optimalPath[counter]
                break

            time.sleep(0.1) # Pause to make animation easier to view
            allowExit() # Allow user to press ESC to exit at any time



        allowExit()



if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
