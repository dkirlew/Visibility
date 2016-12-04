import numpy as np
import random, math
import time
import pygame

class PlanningEnvironment(object):

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

    def CheckCollision(env_config, x, y):
        global height
        global width
        global robot_radius
        
        for shape in env_config:
            if shape[0].lower() == "r":
                if CheckRobotRectangleCollision(shape, x, y):
                    return True
            elif shape[0].lower() == "l":
                if CheckRobotLineCollision(shape, x, y):
                    return True
            elif shape[0].lower() == "c":
                if CheckRobotCircleCollision(shape, x, y):
                    return True
            elif shape[0].lower() == "a":
                if CheckRobotArcCollision(shape, x, y):
                    return True
            else:
                print "Error checking collision for shape type \"" + shape[0] + "\".  Expecting R, L, C, or A.  Exiting."
                exit(0)

        return False


    def CheckRobotRectangleCollision(shape, x, y):
        # Rectangle contain top left, top right, bottom right, and bottom left coordinates    # Line contains start and end coordinate
        global height
        global width
        global robot_radius
        
        print "rectangle collision"
        x1 = shape[1][0] # top left x 
        y1 = -1*(shape[1][1]) # top left y, y is inverted because origin in pygame is top left and origin in regular coordinate system is bottom left
        x2 = shape[2][0] # top right x 
        y2 = -1*(shape[2][1]) # top right y, y is inverted because origin in pygame is top left and origin in regular coordinate system is bottom left
        x3 = shape[3][0] # bottom right x 
        y3 = -1*(shape[3][1]) # bottom right y , y is inverted because origin in pygame is top left and origin in regular coordinate system is bottom left
        x4 = shape[4][0] # bottom left x 
        y4 = -1*(shape[4][1]) # bottom left y, y is inverted because origin in pygame is top left and origin in regular coordinate system is bottom left
        robot_x = x
        robot_y = y * -1 # y is inverted because origin in pygame is top left and origin in regular coordinate system is bottom left


        # print "robot_x =",robot_x
        # print "robot_y =",robot_y
        # print"x1 =",x1
        # print"y1 =",y1
        # print"x2 =",x2
        # print"y2 =",y2
        # print"x3 =",x3
        # print"y3 =",y3
        # print"x4 =",x4
        # print"y4 =",y4

        # first check if robot is within rectangle
        # http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
        # check if point is to right of LEFT edge of rectangle
        Al = -(y4 - y1)
        Bl = x4 - x1
        Cl = -(Al * x1 + Bl * y1)
        Dl = Al * robot_x + Bl * robot_y + Cl
        Ml = -Al/Bl
        # print "Ml:",Ml
        # print "Dl:",Dl

        if Dl == 0: # on the line
            return True
        if (Dl < 0 and Ml > 0) or (Dl > 0 and Ml < 0): # lies to the right (might be in collision)
            # check if point is to the left of the RIGHT edge of rectangle
            Ar = -(y3 - y2)
            Br = x3 - x2
            Cr = -(Ar * x2 + Br * y2)
            Dr = Ar * robot_x + Br * robot_y + Cr
            Mr = -Ar / Br
            # print "Mr:",Mr
            # print "Dr:",Dr

            if Dr == 0: # on the line
                return True 
            # http://math.stackexchange.com/questions/324589/detecting-whether-a-point-is-above-or-below-a-slope
            if (Dr < 0 and Mr < 0) or (Dr > 0 and Mr > 0): # lies to the left (might be in collision)
                # check if the point is below the TOP edge of rectangle (keep in mind that the rectangle is now flipped below origin.  This uses the new top of the rectangle)
                At = -(y2 - y1)
                Bt = x2 - x1
                Ct = -(At * x1 + Bt * y1)
                Dt = At * robot_x + Bt * robot_y + Ct
                # print "Dt:",Dt

                if Dt == 0: # on the line
                    return True
                if Dt < 0: # lies below (might be in collision)
                    # check if point is above the BOTTOM edge of rectangle
                    Ab = -(y4 - y1)
                    Bb = x4 - x1
                    Cb = -(Ab * x1 + Bb * y1)
                    Db = Ab * robot_x + Bb * robot_y + Cb
                    # print "Db:",Db
                    if Db >= 0: # on or above line
                        return True

        # maybe do this instead? http://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/ 

        # next check if robot is within robot_radius of edge of rectangle
        # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line

        dist_left_side = DistPointToLine(robot_x, robot_y, x1, y1, x4, y4)
        dist_right_side = DistPointToLine(robot_x, robot_y, x2, y2, x3, y3)
        dist_top_side = DistPointToLine(robot_x, robot_y, x3, y3, x4, y4)
        dist_bottom_side = DistPointToLine(robot_x, robot_y, x1, y1, x2, y2)

        # print "dist_left_side:",dist_left_side
        # print "dist_right_side:",dist_right_side
        # print "dist_top_side:",dist_top_side
        # print "dist_bottom_side:",dist_bottom_side

        if dist_left_side < robot_radius or dist_right_side < robot_radius or dist_top_side < robot_radius or dist_bottom_side < robot_radius:
            return True


        return False

    def DistPointToLine(point_x, point_y, line1_x, line1_y, line2_x, line2_y):
        numerator = abs((line2_y - line1_y) * point_x - (line2_x - line1_x) * point_y + line2_x * line1_y - line2_y * line1_x)
        denominator = numpy.sqrt(numpy.square(line2_x - line1_x) + numpy.square(line2_y - line1_y))
        dist = numerator / denominator

        return dist


    def CheckRobotLineCollision(shape, x, y):
        # Line contains start and end coordinate
        global height
        global width
        global robot_radius
        
        print "line collision"
        sx = shape[1][0] # start x
        sy = shape[1][1] # start y
        ex = shape[2][0] # end x
        ey = shape[2][1] # end y

        # create rectangle by padding line with robot_radius.  Check for collision between new rectangle and robot


        
        dist = DistPointToLine(x, y, sx, sy, ex, ey)

        if dist < robot_radius: # inside bar around line (line extends between infinities)
            dist_start = numpy.sqrt(numpy.square(sx - x) + numpy.square(sy - y))
            dist_end = numpy.sqrt(numpy.square(ex - x) + numpy.square(ey - y))
            if (x > min(sx, ex) and x < max(sx, ex)) or (y > min(sy, ey) and y < max(sy, ey)):
                return True
            elif dist_start < robot_radius or dist_end < robot_radius:
                return True

        return False


    def CheckRobotCircleCollision(shape, x, y):
        # Circle contains coordinate of center and radius
        global height
        global width
        global robot_radius
        
        cx = int(shape[1][0]) # center x
        cy = int(shape[1][1])# center y
        radius = shape[2] + robot_radius

        # check if distance from center to robot is less than radius of circle plus robot radius
        dist = numpy.sqrt(numpy.square(cx - x) + numpy.square(cy - y))
        if dist < radius:
            return True

        return False


    def CheckRobotArcCollision(shape, x, y):
        # Arc contains coordinate of center, width and height of rectangle containing the arc, start angle, and stop angle.  When added to env_config, Arc will also contain coordinate of center of subtended arc
        global height
        global width
        global robot_radius
        
        robot_x = x
        robot_y = -1 * y

        shape_radius = int(shape[2])
        start_angle = shape[3]
        stop_angle = shape[4]

        arc_x = int(shape[1][0]) + shape_radius # center coordinate x (shape contains top left coordinate.  adding shape_radius creates arc origin/center)
        arc_y = -1 * (int(shape[1][1]) + shape_radius) # center coordinate y (shape contains top left coordinate.  adding shape_radius creates arc origin/center)

        delta_theta = 2*math.sin(float(robot_radius/2)/shape_radius)
        new_start_angle = start_angle - delta_theta
        new_stop_angle = stop_angle + delta_theta

        dist = numpy.sqrt(float(numpy.square(arc_x - robot_x) + numpy.square(arc_y - robot_y)))

        # print "shape_radius:",shape_radius
        # print "start_angle:",start_angle
        # print "stop_angle:",stop_angle
        # print "arc_x:",arc_x
        # print "arc_y:",arc_y
        # print "robot x:",robot_x
        # print "robot y:",robot_y
        # print "dist:",dist

        # check if robot is in "crust" of arc
        if dist < (shape_radius + robot_radius) and dist > (shape_radius - robot_radius):
            # check if robot is inside of "slice" of arc or outside "pie" of arc (slice is empty portion of arc, pie is full arc portion)

            # calculate angle from center of arc to robot
            if arc_y != robot_y:
                robot_theta = math.atan(float(robot_y - arc_y) / float(robot_x - arc_x)) # radians
            else:
                if robot_x > arc_x: #robot is to right of arc
                    robot_theta = 0
                elif robot_x < arc_x: # robot is to left of arc
                    robot_theta = math.pi
                else: # same center
                    if robot_radius >= shape_radius: # robot is huge and eating the arc
                        print "robot eating arc"
                        return True
                    else: # robot fits in arc
                        return False

            # theta is between -pi/4 and pi/4.  need to change to be between 0 and 2pi
            if robot_x < arc_x: # quadrant 2 and 3
                robot_theta = robot_theta + math.pi
            elif robot_y < arc_y: # quadrant 4
                robot_theta = robot_theta + 2*math.pi

            if new_start_angle < math.pi and new_stop_angle > math.pi: # if angle 0 bisects arc slice
                if robot_theta <= new_start_angle or robot_theta >= new_stop_angle: # if in slice
                    return False 
                else:
                    return True
            else: # normal case
                if robot_theta >= new_stop_angle and robot_theta <= new_start_angle: # in slice
                    return False
                else:
                    return True
        else:
            return False


    def InitializePlot(self, env_config, start_config, goal_config, width, height, robot_radius):
        # R = Rectangle, L = Line, C = Circle, A = Arc, S = Start Config, G = Goal Config
        # Rectangle contain top left, top right, bottom right, and bottom left coordinates
        # Line contains start and end coordinate
        # Circle contains coordinate of center and radius
        # Arc contains coordinate of center, width and height of rectangle containing the arc, start angle, and stop angle.  When added to env_config, Arc will also contain coordinate of center of subtended arc
        # Start config contains x and y coordinates of robot and theta of robot config
        # Goal config contains x and y coordinates of robot and theta of robot config

        pygame.init()
        self.screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF)
        self.background = pygame.Surface(self.screen.get_size()).convert()
        self.background.fill((255, 255, 255))
        # self.background = self.background.convert()
        # self.screen.blit(self.background, (0, 0))
        self.clock = pygame.time.Clock()
        self.fps = 30
        self.playtime = 0.0

        print "robot start config"
        start_robot_x = int(start_config[1]) # robot x
        start_robot_y = int(start_config[2]) # robot y
        start_theta = float(start_config[3])# * 180 / math.pi

        # find where ID dot of robot should be to indicate orientation
        # http://math.libretexts.org/Core/Calculus/Precalculus/Chapter_5%3A_Trigonometric_Functions_of_Angles/5.3_Points_on_Circles_using_Sine_and_Cosine
        start_theta_x = int((robot_radius / 2) * math.cos(start_theta)) + start_robot_x
        start_theta_y = -1 * int((robot_radius / 2) * math.sin(start_theta)) + start_robot_y # inverting y because origin for pygame is at top left, origin in coordinate system is in bottom left


        pygame.draw.circle(self.background, self.GREEN, (start_robot_x, start_robot_y), robot_radius)
        pygame.draw.circle(self.background, self.BLACK, (start_theta_x, start_theta_y), robot_radius/4)

        print "robot goal config"
        goal_robot_x = int(goal_config[1]) # robot x
        goal_robot_y = int(goal_config[2]) # robot y
        goal_theta = float(goal_config[3])# * 180 / math.pi

        # find where ID dot of robot should be to indicate orientation
        # http://math.libretexts.org/Core/Calculus/Precalculus/Chapter_5%3A_Trigonometric_Functions_of_Angles/5.3_Points_on_Circles_using_Sine_and_Cosine
        goal_theta_x = int((robot_radius / 2) * math.cos(goal_theta)) + goal_robot_x
        goal_theta_y = -1 * int((robot_radius / 2) * math.sin(goal_theta)) + goal_robot_y # inverting y because origin for pygame is at top left, origin in coordinate system is in bottom left

        pygame.draw.circle(self.background, self.RED, (goal_robot_x, goal_robot_y), robot_radius)
        pygame.draw.circle(self.background, self.BLACK, (goal_theta_x, goal_theta_y), robot_radius/4)
                


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
                print "arc"

                x = int(obstacle[1][0]) # top left x
                y = int(obstacle[1][1]) # top left y
                diameter = int(obstacle[2]) * 2 # stored as radius
                start_angle = obstacle[3] 
                stop_angle = obstacle[4] 

                if start_angle < math.pi and stop_angle > math.pi: # if the circle does not wrap around the origin
                    pygame.draw.arc(self.background, self.BLUE, [x, y, diameter, diameter], start_angle, stop_angle)
                else: # circle wraps around origin: draw arc from 0 to stop_angle and from start_angle to 2*pi
                    pygame.draw.arc(self.background, self.BLUE, [x, y, diameter, diameter], 0, stop_angle)
                    pygame.draw.arc(self.background, self.BLUE, [x, y, diameter, diameter], start_angle, 2*math.pi)

            else:
                print "Unknown descriptor \"" + obstacle[0] + "\".  Expecting R, L, C, A, S, or G.  Exiting"
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

