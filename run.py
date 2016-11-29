#!/usr/bin/env python

import argparse, numpy, openravepy, time, math, random, ast

# from HerbRobot import HerbRobot
# from SimpleRobot import SimpleRobot
# from HerbEnvironment import HerbEnvironment
# from SimpleEnvironment import SimpleEnvironment
from PlanningEnvironment import PlanningEnvironment

# from VisibilityPlanner import VisibilityPlanner
# from RRTPlanner import RRTPlanner
# from PRMPlanner import PRMPlanner
# from VRRTPlanner import VRRTPlanner
# from VPRMPlanner import VPRMPlanner

def main(planner, planning_env, visualize, output, domain):
    height = 1000
    width = 1000
    robot_radius = 100

    env_config = parse_domain(domain, height, width, robot_radius)#numpy.array(robot.GetCurrentConfiguration())
    print "env_config:"
    for config in env_config:
        print str(config)
    # if robot.name == 'herb':
    #     goal_config = numpy.array([ 3.68, -1.90,  0.00,  2.20,  0.00,  0.00,  0.00 ])
    # else:
    #     goal_config = numpy.array([2.0, -0.8])
    planning_env.InitializePlot(env_config, width, height)
    start_time = time.time()
    # # plan = planner.Plan(start_config, goal_config, visualize, output)
    # plan_short = planning_env.ShortenPath(plan)
    print "Time to plan with ",planner,": ",(time.time()-start_time)
    # # traj = robot.ConvertPlanToTrajectory(plan)
    # # robot.ExecuteTrajectory(traj)


#returns env_config list = [name of env (file name), [shape1 type, shape1 info 1, ...], [shape2 type, shape2 info 1, ....], ...]
def parse_domain(domain_file_name, height, width, robot_radius):
    # R = Rectangle, L = Line, C = Circle, A = Arc
    # Rectangles contain top left and bottom right coordinate
    # Line contains start and end coordinate
    # Circle contains coordinate of center and radius
    # Arc contains coordinate of center, width and height of rectangle containing the arc, start angle, and stop angle.  When added to env_config, Arc will also contain coordinate of center of subtended arc

    #TODO: add/prase start and goal configs!! (x,y,theta)

    env_config = []
    if domain_file_name != "r": # if not random, parse file
        for line in open(domain_file_name+str(".txt")):
            line = str(line)
            print "line:",line
            shape_env_config = []
            print "eval",line[0]
            if line[0].lower() == "r" or line[0].lower() =="l": # rectangle and line are [name, coord1, coord2] so can be parsed the same
                split = line.split("|")  # must split on special char | - comma used for coordinates
                print "split:",split
                shape_env_config.append(split[0]) # shape type
                for entry in range(1,len(split)-1): # first entry is shape ID, last is new line character
                    print "entry",split[entry]
                    coord = ast.literal_eval(split[entry])
                    print "coord:",coord
                    #TODO: for rectangle, should add two implied coordinates or leave it at corners?
                    shape_env_config.append([float(coord[0]),float(coord[1])])

            elif line[0].lower() == "c":
                split = line.split("|")  # must split on special char | - comma used for coordinates
                shape_env_config.append(split[0]) # shape type
                coord = ast.literal_eval(split[1])
                shape_env_config.append([float(coord[0]),float(coord[1])])
                shape_env_config.append(float(split[2])) #radius

            elif line[0].lower() == "a":
                split = line.split("|")  # must split on special char | - comma used for coordinates
                shape_env_config.append(split[0]) # shape type
                coord = ast.literal_eval(split[1])
                coord = [float(coord[0]), float(coord[1])]
                radius = float(split[2]) #radians
                start_angle = float(split[3])
                stop_angle = float(split[4])
                angle = stop_angle - start_angle
                bi_angle = start_angle
                bi_angle+= angle/2.0

                # TODO - will need this for sampling node coords
                # bi_x = coord[0]+radius*math.cos(bi_angle)
                # bi_y = coord[1]+radius*math.sin(bi_angle)
                # bi_coord = [bi_x, bi_y]

                shape_env_config.append(coord) # center coordinate
                shape_env_config.append(radius)
                shape_env_config.append(start_angle)
                shape_env_config.append(stop_angle)
                # shape_env_config.append(bi_coord)

            else:
                print "Unknown descriptor \"" + line[0] + "\" while loading",domain_file_name,".  Expecting R, L, C, or A.  Exiting"
                exit(0)
            
            env_config.append(shape_env_config)
    else:
        difficulty = random.randint(1,3) # easy (rectangles and lines), medium (rectangles, lines, circles), hard, (rectangles, lines, circles, arcs)
        if difficulty == 1:
            name = "easy"
        elif difficulty == 2:
            name = "medium"
        elif difficulty == 3:
            name = "hard"
        name+=str(int(time.time())) + ".txt"
        newfile = open(name, "w") #w = write access

        for i in range(difficulty*2): #twice as many shapes as the difficulty
            shape = 3 # random.randint(1,4)%(difficulty + 1)

            if shape == 0: #rectangle
                shape_env_config = CreateRectangle(height, width)

            elif shape == 1: #line
                shape_env_config = CreateLine(height, width)

            elif shape == 2: #circle
                shape_env_config = CreateCircle(height, width)

            else: #arc
                shape_env_config = CreateArc(height, width)

            # TODO: check that shape being added doesn't intersect with existing shapes
            env_config.append(shape_env_config)

            # env_config.append(CreateRobotConfig(env_config, height, width, robot_radius)) # start_config
            # env_config.append(CreateRobotConfig(env_config, height, width, robot_radius)) # goal_config

            for config in shape_env_config:
                newfile.write(str(config) + "|")
            newfile.write("\n")
        newfile.close()
        print "Created new problem file",name,"."
    return env_config


def CreateRectangle(height, width):
    rect_config = ["R"]
    x1 = random.randint(0, width)
    x2 = random.randint(0, width)
    while abs(x2 - x1) < width/10: #TODO: arbitrary
        x2 = random.randint(0, width)

    # ensures rectangle is of decent dimensions (at least 100, no more than 500)
    while abs(x2 - x1) > width/2 or abs(x2 - x1) < width/10:
        x1 = random.randint(0, width)
        while abs(x2 - x1) < width/10: #TODO: arbitrary
            x2 = random.randint(0, width)

    y1 = random.randint(0, height)
    y2 = random.randint(0, height)
    while abs(y2-y1) < height/10: #TODO: arbitrary
        y2 = random.randint(0, height)

    # ensures rectangle is of decent dimensions (at least 100, no more than 500)
    while abs(y2 - y1) > height/2 or abs(y2 - y1) < height/10:
        y1 = random.randint(0, height)
        while abs(y2-y1) < height/10: #TODO: arbitrary
            y2 = random.randint(0, height)

    xa = min(x1, x2) # top left with origin at top left
    ya = min(y1, y2)
    xc = max(x1, x2) # bottom right with origin at top left
    yc = max(y1, y2)

    xb = xc
    yb = ya
    xd = xa
    yd = yc

    # origin coordinate
    ox = xa + (xb - xa)/2
    oy = ya + (yd - ya)/2

    theta = random.randint(0, 90) # rotate no more than 90 degrees to ensure points remain in order of top left, top right, bottom right, bottom left

    x1 = math.cos(theta) * (xa - ox) - math.sin(theta) * (ya - oy) + ox
    y1 = math.sin(theta) * (xa - ox) + math.cos(theta) * (ya - oy) + oy
    x2 = math.cos(theta) * (xb - ox) - math.sin(theta) * (yb - oy) + ox
    y2 = math.sin(theta) * (xb - ox) + math.cos(theta) * (yb - oy) + oy
    x3 = math.cos(theta) * (xc - ox) - math.sin(theta) * (yc - oy) + ox
    y3 = math.sin(theta) * (xc - ox) + math.cos(theta) * (yc - oy) + oy
    x4 = math.cos(theta) * (xd - ox) - math.sin(theta) * (yd - oy) + ox
    y4 = math.sin(theta) * (xd - ox) + math.cos(theta) * (yd - oy) + oy

    rect_config.append([x1, y1]) #top left
    rect_config.append([x2, y2]) #top right
    rect_config.append([x3, y3]) #bottom right
    rect_config.append([x4, y4]) #bottom left

    return rect_config


def CreateLine(height, width):
    line_config = ["L"]
    x1 = random.randint(0, width)
    y1 = random.randint(0, height)
    x2 = random.randint(0, width)
    y2 = random.randint(0, height)

    while abs(x2-x1) < width/5: #TODO: arbitrary
        x2 = random.randint(0, width)
    while abs(y2-y1) < height/5: #TODO: arbitrary
        y2 = random.randint(0, height)

    # ensures line isn't too long or too short
    dist = numpy.sqrt(numpy.square(x1 - x2) + numpy.square(y1 - y2))
    while dist > max(width, height)/2 or dist < min(width, height)/10:
        print dist
        x1 = random.randint(0, width)
        y1 = random.randint(0, height)
        x2 = random.randint(0, width)
        y2 = random.randint(0, height)

        while abs(x2-x1) < width/5: #TODO: arbitrary
            x2 = random.randint(0, width)
        while abs(y2-y1) < height/5: #TODO: arbitrary
            y2 = random.randint(0, height)
        dist = numpy.sqrt(numpy.square(x1 - x2) + numpy.square(y1 - y2))

    line_config.append([x1, y1])
    line_config.append([x2, y2])

    return line_config

def CreateCircle(height, width):
    circle_config = ["C"]
    cx1 = random.randint(0, width)
    cy1 = random.randint(0, height)
    radius = random.randint(min(width, height)/10, min(width, height)/4) # TODO: arbitrary

    # radius is too big or center coordinates too close to edge, so re-roll
    while (cx1 + radius > width) or (cy1 + radius > height) or (cx1 - radius < 0) or (cy1 - radius < 0):
        cx1 = random.randint(0, width)
        cy1 = random.randint(0, height)
        radius = random.randint(min(width, height)/10, min(width, height)/4) # TODO: arbitrary

    circle_config.append([cx1, cy1])
    circle_config.append(radius)

    return circle_config

def CreateArc(height, width):
    arc_config = ["A"]
    x = random.randint(0, width)
    y = random.randint(0, height)
    radius = random.randint(width/10, width/4) # TODO: arbitrary

    # radius is too big or center coordinates too close to edge, so re-roll
    while (x + radius) > width or (x - radius) < 0 or (y + radius) > height or (y - radius) < 0:
        x = random.randint(0, width)
        y = random.randint(0, height)
        radius = random.randint(width/10, width/4) # TODO: arbitrary


    start_angle = round(random.uniform(0, 2*math.pi), 2)
    stop_angle = round(random.uniform(0, 2*math.pi), 2)

    # ensures that angles are unique and arc will be greater than a half circle to create pockets/local minima
    while abs(stop_angle - start_angle) < math.pi or start_angle == stop_angle: # TODO: arbitrary:
        start_angle = round(random.uniform(0, 2*math.pi), 2)
        stop_angle = round(random.uniform(0, 2*math.pi), 2)

    arc_config.append([x, y])
    arc_config.append(radius)
    arc_config.append(min(start_angle, stop_angle)) # start angle
    arc_config.append(max(start_angle, stop_angle)) # stop angle

    return arc_config


def CreateRobotConfig(env_config, height, width, robot_radius):
    # TODO
    config = []


    theta = round(random.uniform(0, 2*math.pi), 2)
    config.append(theta)
    return config



def CheckCollision(env_config, x, y, robot_radius):
    for shape in env_config:
        if shape[0].lower() == "r":
            if CheckRectangleCollision(shape, x, y, robot_radius):
                return True
        elif shape[0].lower() == "l":
            if CheckLineCollision(shape, x, y, robot_radius):
                return True
        elif shape[0].lower() == "c":
            if CheckCircleCollision(shape, x, y, robot_radius):
                return True
        elif shape[0].lower() == "a":
            if CheckArcCollision(shape, x, y, robot_radius):
                return True
        else:
            print "Error checking collision for shape type \"" + shape[0] + "\".  Expecting R, L, C, or A.  Exiting."
            exit(0)

    return False


def CheckRectangleCollision(shape, x, y, robot_radius):
    x1 = shape[1][0] + robot_radius # top left x + robot_radius to create buffer
    y1 = shape[1][1] + robot_radius # top left y + robot_radius to create buffer
    x2 = shape[2][0] + robot_radius # top right x + robot_radius to create buffer
    y2 = shape[2][1] + robot_radius # top right y + robot_radius to create buffer
    x3 = shape[3][0] + robot_radius # bottom right x + robot_radius to create buffer
    y3 = shape[3][1] + robot_radius # bottom right y + robot_radius to create buffer
    x4 = shape[4][0] + robot_radius # bottom left x + robot_radius to create buffer
    y4 = shape[4][1] + robot_radius # bottom left y + robot_radius to create buffer

    # http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
    # check if point is to right of LEFT edge of rectangle
    Al = -(y4 - y1)
    Bl = x4 - y1
    Cl = -(Al * y1 + Bl * y1)
    Dl = Al * x + Bl * y + Cl

    if Dl == 0: # on the line
        return True
    if Dl > 0: # lies to the right (might be in collision)
        # check if point is to the left of the RIGHT edge of rectangle
        Ar = -(y3 - y2)
        Br = x3 - x2
        Cr = -(Ar * x2 + Br * y2)
        Dr = Ar * x + Br * y + C

        if Dr == 0: # on the line
            return True 
        # http://math.stackexchange.com/questions/324589/detecting-whether-a-point-is-above-or-below-a-slope
        if Dr < 0: # lies to the left
            # check if the point is below the TOP edge of rectangle
            At = -(y2 - y1)
            Bt = x2 - x1
            Ct = -(At * x1 + Bt * y1)
            Dt = At * x + Bt * y + C
            if Dt == 0: # on the line
                return True
            if Dr < 0: # lies below
                # check if point is above the BOTTOM edge of rectangle
                Ab = -(y4 - y3)
                Bb = x4 - x3
                Cb = -(A * x3 + B * y3)
                Db = Ab * x + Bb * y + C
                if Dt >= 0: # on or above line
                    return True

    # maybe do this instead? http://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/ 

    return False


def CheckLineCollision(shape, x, y, robot_radius):
    sx = shape[1][0] # start x
    sy = shape[1][1] # start y
    ex = shape[2][0] # end x
    ey = shape[2][1] # end y

    # create rectangle by padding line with robot_radius.  Check for collision between new rectangle and robot

    rect_config = ["R"]
    if sx == min(sx, ex): # if start of line is the left-most point, save as left x and y, save end as right x and y
        lx = sx
        ly = sy
        rx = ex
        ry = ey
    else: # do opposite
        lx = ex
        ly = ey
        rx = sx
        ry = sy

    rect_config.append([lx - robot_radius, ly - robot_radius]) # top left
    rect_config.append([rx + robot_radius, ry - robot_radius]) # top right
    rect_config.append([rx + robot_radius, ry + robot_radius]) # bottom right
    rect_config.append([lx + robot_radius, ly + robot_radius]) # bottom left

    return CheckRectangleCollision(rect_config, x, y, robot_radius)


def CheckCircleCollision(shape, x, y, robot_radius):
    cx = int(shape[1][0]) # center x
    cy = int(shape[1][1])# center y
    radius = shape[2] + robot_radius

    # check if distance from center to robot is less than radius of circle plus robot radius
    dist = numpy.sqrt(numpy.square(cx - x) + numpy.square(cy - y))
    if dist < radius:
        return True

    return False


def CheckArcCollision(shape, x, y, robot_radius):
    arc_x = int(shape[1][0]) # center coordinate x
    arc_y = int(shape[1][1]) # center coordinate y
    shape_radius = int(shape[2])
    start_angle = shape[3]
    stop_angle = shape[4]

    #arc_length = radius * abs(start_angle - stop_angle)

    delta_theta = 2*sin((robot_radius/2)/shape_radius)
    new_start_angle = start_angle - delta_theta
    new_stop_angle = stop_angle + delta_theta

    dist = numpy.sqrt(numpy.square(arc_x - x) + numpy.square(arc_y - y))

    # check if robot is in "crust" of arc
    if dist < (shape_radius + robot_radius) or dist > (shape_radius - robot_radius):
        # check if robot is inside of "slice" of arc or outside "pie" of arc (slice is empty portion of arc, pie is full arc portion)

        # calculate angle from center of arc to robot
        robot_theta = math.atan((x - arc_x) / (y - arc_y)) # degrees
        robot_theta = robot_theta * math.pi / 180 # degrees to radians

        # theta is between -pi/4 and pi/4.  need to change to be between 0 and 2pi
        if x < arc_x: # quadrant 2 and 3
            robot_theta = robot_theta + math.pi
        elif y < arc_y: # quadrant 4
            robot_theta = robot_theta + 2*math.pi

        if start_angle < math.pi and stop_angle > math.pi: # if angle 0 bisects arc slice
            if robot_theta <= start_angle or robot_theta => stop_angle: # if in slice
                return False 
            else:
                return True
        else: # normal case
            if robot_theta => stop_angle and robot_theta <= start_angle: # in slice
                return False
            else:
                return True
    else:
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='script for testing planners')
    
    # parser.add_argument('-r', '--robot', type=str, default='simple',
    #                     help='The robot to load (herb or simple)')
    parser.add_argument('-p', '--planner', type=str, default='v',
                        help='The planner to run (v,prm,rrt,vprm,vrrt)')
    # parser.add_argument('-m', '--manip', type=str, default='right',
    #                     help='The manipulator to plan with (right or left) - only applicable if robot is of type herb')
    parser.add_argument('-v', '--visualize', default='True',
                        help='Enable visualization of graph (default is True')
    parser.add_argument('-o', '--output', default='False',
                        help='Enable output logging (default is False)')
    parser.add_argument('-d', '--domain', default = 'r',
                        help='Select which domain to use (r for random, or enter file name.  Default is random)')
    

    args = parser.parse_args()
    
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    # openravepy.misc.InitOpenRAVELogging()

    # if args.debug:
    #     openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    # env = openravepy.Environment()
    # env.SetViewer('qtcoin')
    # env.GetViewer().SetName('Visibility Adventerous Graph')

    # First setup the environment and the robot
    visualize = args.visualize
    domain = args.domain
    #TODO
    # if args.robot == 'herb':
    #     robot = HerbRobot(env, args.manip)
    #     planning_env = HerbEnvironment(robot)
    #     visualize = False
    # elif args.robot == 'simple':
    #     robot = SimpleRobot(env)
    #     planning_env = SimpleEnvironment(robot)
    # else:
    #     print 'Unknown robot option: %s' % args.robot
    #     exit(0)
    # Next setup the planner
    planning_env = PlanningEnvironment()
    planner = args.planner
    # # if args.planner == 'v':
    # #     planner = VisibilityPlanner(planning_env, visualize=visualize)
    # # elif args.planner == 'rrt':
    # #     planner = RRTPlanner(planning_env, visualize=visualize)
    # # elif args.planner == 'prm':
    # #     planner = PRMPlanner(planning_env, visualize=visualize)
    # # elif args.planner == 'vrrt':
    # #     planner = VRRTPlanner(planning_env, visualize=visualize)
    # # elif args.planner == 'vprm':
    # #     planner = VPRMPlanner(planning_env, visualize=visualize)
    # # else:
    # #     print 'Unknown planner option: %s' % args.planner
    # #     exit(0)

    main(planner, planning_env, visualize, None, domain)

    # import IPython
    # IPython.embed()