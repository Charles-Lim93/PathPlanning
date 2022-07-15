import sys, random, math, pygame
from pygame.locals import *
from math import sqrt, cos, sin, atan2
from lineIntersect import *

# MAP SIZE DECLARATION
XDIM = 700
YDIM = 500
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 300
RADIUS = 15
OBS = [(500,150,100,50),(300,80,100,50),(150,220,100,50), (300, 50, 75, 75), (100, 200, 50, 100), (30, 20, 50,50), (450, 350, 75, 75)]
dynamic_OBS = [(350, 250)] # dynamic obstacle detection

def obsDraw(pygame, screen):
    blue = (0,0,255)
    for o in OBS:
        pygame.draw.rect(screen, blue, o)
    
def dynamicObsDraw(pygame, screen):
    green = (0,255, 0)
    for o in dynamic_OBS:
        pygame.draw.circle(screen, green, o, 40)
        
def dist(p1, p2):
    return sqrt((p1[0] - p2[0])*(p1[0] - p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1], p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)
    
def chooseParent(nn, newnode, nodes):
    for p in nodes:
        if checkIntersect(p,newnode, OBS) and dist([p.x, p.y], [newnode.x, newnode.y]) < RADIUS and p.cost + dist([p.x, p.y], [newnode.x, newnode.y]) < nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y]):
            nn = p
        newnode.cost = nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y])
        newnode.parent = nn
        return newnode, nn
    
def reWire(nodes, newnode, pygame, screen):
    white = 255, 240, 200
    black = 20, 20, 40
    for i in range(len(nodes)):
        p = nodes[i]
        if checkIntersect(p,newnode, OBS) and p != newnode.parent and dist([p.x, p.y],[newnode.x, newnode.y]) < RADIUS and newnode.cost+dist([p.x, p.y], [newnode.x, newnode.y]) < p.cost:
            pygame.draw.line(screen, white, [p.x, p.y], [p.parent.x, p.parent.y])
            p.parent = newnode
            p.cost = newnode.cost + dist([p.x, p.y], [newnode.x, newnode.y])
            nodes[i] = p
            pygame.draw.line(screen, black, [p.x, p.y], [newnode.x, newnode.y])
    return nodes

def drawSolutionPath(start, goal, nodes, pygame, screen):
    pink = 200, 20, 240
    nn = nodes[0]
    final_path = []
    k = 0
    print("nodes are: ", nodes)
    for p in nodes:
        if dist([p.x, p.y], [goal.x, goal.y]) < dist([nn.x, nn.y], [goal.x, goal.y]):
            nn = p
    while nn != start:
        pygame.draw.line(screen, pink, [nn.x, nn.y], [nn.parent.x, nn.parent.y], 5)
        nn = nn.parent
        print("Optimal Solution Nodes:",[nn.x, nn.y])
        final_path.append(nn.x)
    return final_path


class Cost:
    x = 0
    y = 0
    cost = 0
    parent = None
    def __init__(self, xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord
                
class Node:
    x = 0
    y = 0
    cost = 0
    parent=None
    def __init__(self, xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord
    
def main():
    #INITIALIZE AND PREPARE SCRREN
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRT_Star')
    white = 255, 255, 255
    black = 20, 20, 40
    screen.fill(white)
    obsDraw(pygame, screen)
    dynamicObsDraw(pygame, screen)
    
    nodes = []
    nodes.append(Node(0.0, 0.0))
    start = nodes[0]
    goal = Node(600.0, 450.0)
    for i in range(NUMNODES):
        rand = Node(random.random() * XDIM, random.random() * YDIM)
        nn = nodes[0]
        for p in nodes:
            if dist([p.x, p.y], [rand.x, rand.y]) < dist([nn.x, nn.y], [rand.x, rand.y]):
                nn = p
                # print([p.x, p.y])        
        interpolatedNode = step_from_to([nn.x, nn.y],[rand.x, rand.y])
        newnode = Node(interpolatedNode[0], interpolatedNode[1])
        
        if checkIntersect(nn,rand,OBS) == True:
            [newnode, nn] = chooseParent(nn, newnode, nodes);
            nodes.append(newnode)
            pygame.draw.line(screen, black, [nn.x, nn.y], [newnode.x, newnode.y])
            nodes = reWire(nodes, newnode, pygame, screen)
            pygame.display.update()

        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
    result = drawSolutionPath(start,goal,nodes,pygame,screen)
    pygame.display.update()
    print(result)
    # if python says run, then we should run
if __name__ == '__main__':
    main()
    running = True
    while running:
       for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            
            
    