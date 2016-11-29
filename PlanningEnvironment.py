import numpy as np
import random, math
import time
import pygame

class PlanningEnvironment(object):
    global table

    def __init__(self):
        # global table
        # self.robot = herb.robot
        # self.boundary_limits = [[-5., -5.], [5., 5.]]

        # # add an obstacle
        # table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        # self.robot.GetEnv().Add(table)

        # table_pose = np.array([[ 0, 0, -1, 1.0], 
        #                           [-1, 0,  0, 0], 
        #                           [ 0, 1,  0, 0], 
        #                           [ 0, 0,  0, 1]])
        # table.SetTransform(table_pose)

        # # goal sampling probability
        # self.p = 0.0
        self.BLACK = (  0,   0,   0)
        self.WHITE = (255, 255, 255)
        self.BLUE =  (  0,   0, 255)
        self.GREEN = (  0, 255,   0)
        self.RED =   (255,   0,   0)
        return

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        global table
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits

        #
        # TODO: Generate and return a random configuration
        #
        
        found = False

        #print lower_limits
        #print upper_limits
        while found == False:
                x = round(random.uniform(lower_limits[0], upper_limits[0]),3)
                y = round(random.uniform(lower_limits[1], upper_limits[1]),3)
                tempTrans = self.robot.GetTransform()
                self.robot.SetTransform(np.array([[1, 0, 0, x],
                                                  [0, 1, 0, y],
                                                  [0, 0, 1, 0],
                                                  [0, 0, 0, 1]]))
                if self.robot.GetEnv().CheckCollision(self.robot, table) == False:
                    found = True
                    config = [x,y]
                self.robot.SetTransform(tempTrans)
        
        return np.array(config)

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #

        dist = np.sqrt(np.square(start_config[0]-end_config[0])+np.square(start_config[1]-end_config[1]))
        return dist

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        numPartitions = 1000
        x = start_config[0]
        y = start_config[1]

        deltaX = (end_config[0]-x)/numPartitions
        deltaY = (end_config[1]-y)/numPartitions

        
        tempTrans = self.robot.GetTransform()

        for i in range(1,numPartitions):
            x += deltaX
            y += deltaY
            #print i
            self.robot.SetTransform(np.array([[1, 0, 0, x],
                                              [0, 1, 0, y],
                                              [0, 0, 1, 0],
                                              [0, 0, 0, 1]]))
            if self.robot.GetEnv().CheckCollision(self.robot, table) == True:
                x -= deltaX
                y -= deltaY
                i = numPartitions+1
                if (x == start_config[0]) & (y == start_config[1]):
                    return None
                else:
                    return [x, y]
        return end_config

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        start_time = time.time()
        numPartitions = 50
        tempPath = np.array(path[len(path)-1])


        for i in range(len(path)-1,0,-1):
            start_config = path[i-1]
            end_config = path[i]
            x = end_config[0]
            y = end_config[1]
            deltaX = (x-start_config[0])/numPartitions
            deltaY = (y-start_config[1])/numPartitions
            x -= deltaX
            y -= deltaY

            for j in range(0,numPartitions):
                tempPath = np.vstack(([x,y],tempPath))
                x -= deltaX
                y -= deltaY

        #print("len divided path: " + str(len(tempPath)))
        path = tempPath
        
        
        for i in range(0,len(path)-2):
            for j in range(len(path)-1,i+2,-1):
                #print("start i: " + str(i))
                #print("start j: " + str(j))
                extendConfig = self.Extend(path[i],path[j])
                if extendConfig != None:
                    if (extendConfig == path[j]).all():
                        l = i+1
                        for k in range(i+1,j-1):
                            #print("org length " + str(len(path)))
                            #print("deleted node " + str(l))
                            path = np.delete(path,l,0)
                            #print("new length " + str(len(path)))
                        break
                if time.time() - start_time >= timeout:
                    j = i
                    i = len(path)
                #print("end i: " + str(i))
                #print("end j: " + str(j))
        dist = []
        for i in range(0,len(path)-1):
            dist.append(self.ComputeDistance(path[i],path[i+1]))
        print("shortened distance: " + str(sum(dist)))
        print("time to shorten: " + str(time.time() - start_time))
        print("new number of vertices: " + str(len(path)))

        return path


    def InitializePlot(self, env_config, width, height):
        # R = Rectangle, L = Line, C = Circle, A = Arc
        # Rectangles contain top left and bottom right coordinate
        # Line contains start and end coordinate
        # Circle contains coordinate of center and radius
        # Arc contains coordinate of center, width and height of rectangle containing the arc, start angle, and stop angle.  When added to env_config, Arc will also contain coordinate of center of subtended arc

        pygame.init()
        self.screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF)
        self.background = pygame.Surface(self.screen.get_size()).convert()
        self.background.fill((255, 255, 255))
        # self.background = self.background.convert()
        # self.screen.blit(self.background, (0, 0))
        self.clock = pygame.time.Clock()
        self.fps = 30
        self.playtime = 0.0


        # Show all obstacles in environment
        for obstacle in env_config:
            if obstacle[0].lower() == "r":
                print "rectangle"

                x1 = obstacle[1][0] # top left x
                y1 = obstacle[1][1] # top left y
                x2 = obstacle[2][0] # top right x
                y2 = obstacle[2][1] # top right y
                x3 = obstacle[3][0] # bottom right x
                y3 = obstacle[3][1] # bottom right y
                x4 = obstacle[4][0] # bottom left x
                y4 = obstacle[4][1] # bottom left y
                width = obstacle[2][0] - obstacle[1][0]
                height = obstacle[1][1] - obstacle[2][1]

                pygame.draw.polygon(self.background, self.GREEN, ((x1, y1), (x2, y2), (x3, y3), (x4, y4)))
                
            elif obstacle[0].lower() == "l":
                print "line"

                sx = obstacle[1][0] # start x
                sy = obstacle[1][1] # start y
                ex = obstacle[2][0] # end x
                ey = obstacle[2][1] # end y

                pygame.draw.line(self.background, self.BLACK, [sx, sy], [ex, ey])

            elif obstacle[0].lower() == "c":
                print "circle"

                cx = int(obstacle[1][0]) # center x
                cy = int(obstacle[1][1])# center y
                radius = int(obstacle[2])

                pygame.draw.circle(self.background, self.RED, (cx, cy), radius)

            elif obstacle[0].lower() == "a":
                print "Arc"

                x = int(obstacle[1][0]) # top left x
                y = int(obstacle[1][1]) # top left y
                radius = int(obstacle[2])
                start_angle = obstacle[3] 
                stop_angle = obstacle[4] 

                # pygame.draw.arc(self.background, self.BLUE, [x, y, awidth, aheight], start_angle, stop_angle)
                pygame.draw.arc(self.background, self.BLUE, [x, y, radius, radius], start_angle, stop_angle)

            else:
                print "Unknown descriptor \"" + obstacle[0] + "\".  Expecting R, L, C, or A.  Exiting"
                exit(0)
                

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False 
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

            milliseconds = self.clock.tick(self.fps)
            self.playtime += milliseconds / 1000.0
            # self.draw_text("FPS: {:6.3}{}PLAYTIME: {:6.3} SECONDS".format(
                           # self.clock.get_fps(), " "*5, self.playtime))

            pygame.display.flip()
            self.screen.blit(self.background, (0, 0))

        
        # time.sleep(5)
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

