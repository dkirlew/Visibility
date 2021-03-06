#!/usr/bin/env python

import argparse, numpy, time, math, random, ast

from PlanningEnvironment import PlanningEnvironment
from VisibilityPlanner import VisibilityPlanner
from RRTPlanner import RRTPlanner
from PRMPlanner import PRMPlanner
from VRRTPlanner import VRRTPlanner
from VPRMPlanner import VPRMPlanner


def main(planner, planning_env, visualize, domain, planner_name, trials):
    global height
    global width
    global robot_radius

    env_config, start, goal, file_name = parse_domain(domain)#numpy.array(robot.GetCurrentConfiguration())
    print ("env_config:")
    for config in env_config:
        print (str(config))
    print ("start_config:",start)
    print ("goal_config:",goal)

    start_x = start[1]
    start_y = start[2]
    start_theta = start[3]
    goal_x = goal[1]
    goal_y = goal[2]
    goal_theta = goal[3]

    start_config = ((start_x, start_y), start_theta)

    goal_config = ((goal_x, goal_y), goal_theta)

    if not trials:

        if planner_name == 'v':
            planner = VisibilityPlanner(planning_env, visualize, width, height, robot_radius)
        elif planner_name == 'rrt':
            planner = RRTPlanner(planning_env, visualize, width, height, robot_radius)
        elif planner_name == 'prm':
            planner = PRMPlanner(planning_env, visualize, width, height, robot_radius)
        elif planner_name == 'vrrt':
            planner = VRRTPlanner(planning_env, visualize, width, height, robot_radius)
        elif planner_name == 'vprm':
            planner = VPRMPlanner(planning_env, visualize, width, height, robot_radius)
        else:
            print ('Unknown planner option: %s' % args.planner)
            exit(0)

        start_time = time.time()
        
        if planner_name == 'rrt' or planner_name == 'vrrt':
            Vertices, Edges, path, construct_time, node_nums, len_path, if_fail = planner.Plan(env_config, start_config, goal_config)
            total_time = time.time() - start_time
            plan_time = total_time - construct_time
        elif planner_name == 'prm' or planner_name == 'vprm':
            Vertices, Edges, path, prm_times, num_nodes, len_path, if_fail = planner.Plan(env_config, start_config, goal_config)
        else: 
            Vertices, Edges, path, construct_time, num_nodes, len_path, if_fail = planner.Plan(env_config, start_config, goal_config)
            total_time = time.time() - start_time
            plan_time = total_time - construct_time
        
        planning_env.InitializePlot(Vertices, Edges, path, env_config, start_config, goal_config, file_name, planner_name)
        # planning_env.InitializeMiniPlot(env_config, start_config, goal_config, file_name)
        # print "total_time:",total_time
        # print "construct_time:",construct_time
        # print "plan_time:",plan_time
      
        print "planner:",planner_name
        print "file name:",file_name

        if planner_name == 'prm' or planner_name == 'vprm':
            print "total plan time:",prm_times[0]
            print "learn time:",prm_times[1]
            print "query time:",prm_times[2]
            print "construct time:",prm_times[3]
            print "expand time:",prm_times[4]
        else:
            print "plan_time:",plan_time

        if planner_name == 'rrt' or planner_name == 'vrrt':
            print "total num nodes:",node_nums[0]
            print "nodes reached by extension:",node_nums[1]
            print "nodes failed to reach completed:",node_nums[2]
        else:
            print "num_nodes:",num_nodes
        print "len_path:",len_path
        print "if_fail:",if_fail

    else:
        newfile = open("trials" + "-" + file_name + ".txt", "w") #w = write access

        planner_types = {}
        planner_types["V"] = VisibilityPlanner(planning_env, visualize, width, height, robot_radius)
        planner_types["RRT"] = RRTPlanner(planning_env, visualize, width, height, robot_radius)
        planner_types["PRM"] = PRMPlanner(planning_env, visualize, width, height, robot_radius)
        planner_types["VRRT"] = VRRTPlanner(planning_env, visualize, width, height, robot_radius)
        planner_types["VPRM"] = VPRMPlanner(planning_env, visualize, width, height, robot_radius)
        planner_names = ["V", "RRT", "PRM", "VRRT", "VPRM"]
        
        for planner_name in planner_names: # each planner
            planner = planner_types[planner_name]
            num_trials = 50
            if planner_name == "V":
                print "planner:",planner_name
                num_trials = 1

            for i in range(num_trials):
                start_time = time.time()

                if planner_name == 'RRT' or planner_name == 'VRRT':
                    Vertices, Edges, path, construct_time, node_nums, len_path, if_fail = planner.Plan(env_config, start_config, goal_config)
                    total_time = time.time() - start_time
                    plan_time = total_time - construct_time
                elif planner_name == 'PRM' or planner_name == 'VPRM':
                    Vertices, Edges, path, prm_times, num_nodes, len_path, if_fail = planner.Plan(env_config, start_config, goal_config)
                else: 
                    Vertices, Edges, path, construct_time, num_nodes, len_path, if_fail = planner.Plan(env_config, start_config, goal_config)
                    total_time = time.time() - start_time
                    plan_time = total_time - construct_time

                planning_env.InitializePlot(Vertices, Edges, path, env_config, start_config, goal_config, file_name, planner_name, i)


                print "total_time:",total_time
                print "construct_time:",construct_time
                print "plan_time:",plan_time
             
                newfile.write(file_name + "\t" + "trial " + str(i) + "\t")
                newfile.write(str(planner_name) + "\t")


                if planner_name == 'PRM' or planner_name == 'VPRM':
                    newfile.write(str(prm_times[0]) + "\t") #total plan time
                    newfile.write(str(prm_times[1]) + "\t") #learn time
                    newfile.write(str(prm_times[2]) + "\t") #query time
                    newfile.write(str(prm_times[3]) + "\t") #construct time
                    newfile.write(str(prm_times[4]) + "\t") #expand time
                else:
                    newfile.write(str(plan_time) + "\t")
                    newfile.write("\t")
                    newfile.write("\t")
                    newfile.write("\t")
                    newfile.write("\t")
                if planner_name == 'RRT' or planner_name == 'VRRT':
                    newfile.write(str(node_nums[0]) + "\t") # total num nodes
                    newfile.write(str(node_nums[1]) + "\t") # nodes reached by extension
                    newfile.write(str(node_nums[2]) + "\t") # nodes failed to reach completed
                else:
                    newfile.write(str(num_nodes) + "\t")
                    newfile.write("\t")
                    newfile.write("\t")

                newfile.write(str(len_path) + "\t")
                newfile.write(str(if_fail) + "\n")

        newfile.close()
         

#returns env_config list = [name of env (file name), [shape1 type, shape1 info 1, ...], [shape2 type, shape2 info 1, ....], ...]
def parse_domain(domain_file_name):
    # R = Rectangle, L = Line, C = Circle, A = Arc, S = Start Config, G = Goal Config
    # Rectangle contain top left, top right, bottom right, and bottom left coordinates    # Line contains start and end coordinate
    # Line contains start and end coordinate
    # Circle contains coordinate of center and radius
    # Arc contains coordinate of center, width and height of rectangle containing the arc, start angle, and stop angle.  When added to env_config, Arc will also contain coordinate of center of subtended arc
    # Start config contains x and y coordinates of robot and theta of robot config
    # Goal config contains x and y coordinates of robot and theta of robot config
    global height
    global width
    global robot_radius

    env_config = []
    robot_start_config = []
    robot_goal_config = []

    if domain_file_name != "r" and domain_file_name != "easy" and domain_file_name != "medium" and domain_file_name != "hard": # if not random, parse file
        for line in open(domain_file_name+str(".txt")):
            line = str(line)
            # print ("line:",line)
            shape_env_config = []
            # print ("eval",line[0])
            if line[0].lower() == "r" or line[0].lower() =="l": # rectangle and line are [name, coord1, coord2] so can be parsed the same
                split = line.split("|")  # must split on special char | - comma used for coordinates
                # print ("split:",split)
                shape_env_config.append(split[0]) # shape type
                for entry in range(1,len(split)-1): # first entry is shape ID, last is new line character
                    coord = ast.literal_eval(split[entry])
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

            elif line[0].lower() == "s":
                split = line.split("|") 
                robot_start_config.append(split[0])
                x = float(split[1])
                y = float(split[2])
                theta = float(split[3])

                robot_start_config.append(x)
                robot_start_config.append(y)
                robot_start_config.append(theta)

            elif line[0].lower() == "g":
                split = line.split("|") 
                robot_goal_config.append(split[0])
                x = float(split[1])
                y = float(split[2])
                theta = float(split[3])

                robot_goal_config.append(x)
                robot_goal_config.append(y)
                robot_goal_config.append(theta)

            else:
                print ("Unknown descriptor \"" + line[0] + "\" while loading",domain_file_name,".  Expecting R, L, C, A, S, or G.  Exiting")
                exit(0)
            
            if shape_env_config != []:
                env_config.append(shape_env_config)
    else:
        if domain_file_name == "r":
            difficulty = random.randint(1,3) # easy (rectangles and lines), medium (rectangles, lines, circles), hard, (rectangles, lines, circles, arcs)
            if difficulty == 1:
                name = "easy"
            elif difficulty == 2:
                name = "medium"
            elif difficulty == 3:
                name = "hard"
        elif domain_file_name == "easy":
            difficulty = 1
            name = "easy"
        elif domain_file_name == "medium":
            difficulty = 2
            name = "medium"
        elif domain_file_name == "hard":
            difficulty = 3
            name = "hard"
        else:
            print ("Unknown file name \"" + domain_file_name + "\".  Expecting r, easy, medium, hard, or file name.  Exiting.")
            exit(0)

        name+=str(int(time.time()))
        domain_file_name = name
        name+=".txt"
        newfile = open(name, "w") #w = write access


        for i in range(difficulty*2): #twice as many shapes as the difficulty
            # shape = random.randint(1,4)%(difficulty + 1)
            shape = random.randint(1,3)%difficulty
            print "shape:",shape

            if shape == 0: #rectangle
                shape_env_config = CreateRectangle()
                print "rect config:",shape_env_config

            elif shape == 1: #line
                shape_env_config = CreateLine()

            else:
                shape_env_config = CreateCircle()

            env_config.append(shape_env_config)

            for config in shape_env_config:
                newfile.write(str(config) + "|")
            newfile.write("\n")

        robot_start_config = CreateRobotConfig(env_config) # start_config
        robot_goal_config = CreateRobotConfig(env_config, robot_start_config) # goal_config
        for config in robot_start_config:
            newfile.write(str(config) + "|")
        newfile.write("\n")
        for config in robot_goal_config:
            newfile.write(str(config) + "|")
        newfile.write("\n")     

        newfile.close()
        print ("Created new problem file",name,".")
    return env_config, robot_start_config, robot_goal_config, domain_file_name


def CreateRectangle():
    # Rectangle contain top left, top right, bottom right, and bottom left coordinates
    global height
    global width
    global robot_radius

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

    # theta is negative because y is opposite in pygame
    theta = -1 * random.uniform(0, math.pi/2) # rotate no more than 90 degrees to ensure points remain in order of top left, top right, bottom right, bottom left

    x1 = math.cos(theta) * (xa - ox) - math.sin(theta) * (ya - oy) + ox
    y1 = math.sin(theta) * (xa - ox) + math.cos(theta) * (ya - oy) + oy
    x2 = math.cos(theta) * (xb - ox) - math.sin(theta) * (yb - oy) + ox
    y2 = math.sin(theta) * (xb - ox) + math.cos(theta) * (yb - oy) + oy
    x3 = math.cos(theta) * (xc - ox) - math.sin(theta) * (yc - oy) + ox
    y3 = math.sin(theta) * (xc - ox) + math.cos(theta) * (yc - oy) + oy
    x4 = math.cos(theta) * (xd - ox) - math.sin(theta) * (yd - oy) + ox
    y4 = math.sin(theta) * (xd - ox) + math.cos(theta) * (yd - oy) + oy


    rect_config.append([round(x1, 2), round(y1, 2)]) #top left
    rect_config.append([round(x2, 2), round(y2, 2)]) #top right
    rect_config.append([round(x3, 2), round(y3, 2)]) #bottom right
    rect_config.append([round(x4, 2), round(y4, 2)]) #bottom left

    return rect_config


def CreateLine():
    # Line contains start and end coordinate
    global height
    global width
    global robot_radius

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
        x1 = random.randint(0, width)
        y1 = random.randint(0, height)
        x2 = random.randint(0, width)
        y2 = random.randint(0, height)

        while abs(x2-x1) < width/5: #TODO: arbitrary
            x2 = random.randint(0, width)
        while abs(y2-y1) < height/5: #TODO: arbitrary
            y2 = random.randint(0, height)
        dist = numpy.sqrt(numpy.square(x1 - x2) + numpy.square(y1 - y2))

    line_config.append([round(x1, 2), round(y1, 2)])
    line_config.append([round(x2, 2), round(y2, 2)])

    return line_config


def CreateCircle():
    # Circle contains coordinate of center and radius
    global height
    global width
    global robot_radius
    
    circle_config = ["C"]
    cx1 = random.randint(0, width)
    cy1 = random.randint(0, height)
    radius = random.randint(min(width, height)/15, min(width, height)/4) # TODO: arbitrary

    # radius is too big or center coordinates too close to edge, so re-roll
    while (cx1 + radius > width) or (cy1 + radius > height) or (cx1 - radius < 0) or (cy1 - radius < 0):
        cx1 = random.randint(0, width)
        cy1 = random.randint(0, height)
        radius = random.randint(min(width, height)/10, min(width, height)/5) # TODO: arbitrary

    circle_config.append([cx1, cy1])
    circle_config.append(radius)

    return circle_config


def CreateArc():
    # Arc contains coordinate of center, width and height of rectangle containing the arc, start angle, and stop angle.  When added to env_config, Arc will also contain coordinate of center of subtended arc
    global height
    global width
    global robot_radius
    
    arc_config = ["A"]
    x = random.randint(0, width) # left coord of square that contains arc
    y = random.randint(0, height) # top coord of square that contains arc
    radius = random.randint(width/10, width/4) # TODO: arbitrary

    # radius is too big or center coordinates too close to edge, so re-roll
    # keep in mind, center of circle is at (x + radius, y + radius)
    while (x + 2 * radius) > width or (y + 2 * radius) > height:
        x = random.randint(0, width)
        y = random.randint(0, height)
        radius = random.randint(width/10, width/4) # TODO: arbitrary


    angle1 = round(random.uniform(0, 2*math.pi), 2)
    angle2 = round(random.uniform(0, 2*math.pi), 2)

    # ensures that angles are unique and arc will be greater than a half circle to create pockets/local minima
    while abs(angle2 - angle1) < math.pi or angle1 == angle2: # TODO: arbitrary:
        angle1 = round(random.uniform(0, 2*math.pi), 2)
        angle2 = round(random.uniform(0, 2*math.pi), 2)

    # because start is min and stop is max, most arcs have openings towards the right.  by shifting both start and stop
    # angles by X amount, this rotates the opening of the arc
    angle_shift = round(random.uniform(0, 2*math.pi), 2)
    start_angle = (min(angle1, angle2) + angle_shift) % (2 * math.pi)
    stop_angle = (max(angle1, angle2) + angle_shift) % (2 * math.pi)

    arc_config.append([x, y])
    arc_config.append(radius)
    arc_config.append(start_angle) # start angle
    arc_config.append(stop_angle) # stop angle

    return arc_config


def CreateRobotConfig(env_config, start_config = None):
    # Robot config contains x and y coordinates of robot and theta of robot config
    global height
    global width
    global robot_radius
    
    if start_config is None:
        config = ["S"] # start
    else:
        config = ["G"] # goal

    x = random.randint(0 + robot_radius, width - robot_radius)
    y = random.randint(0 + robot_radius, height - robot_radius)
    theta = round(random.uniform(0, 2*math.pi), 2)

    # check if robot config is in collision with obstacles or if it is too close to start config
    if start_config is not None:
        # print ("checking goal collisions")
        start_x = start_config[1]
        start_y = start_config[2]
        dist_to_start = dist = numpy.sqrt(numpy.square(x - start_x) + numpy.square(y - start_y))
        while (CheckCollision(env_config, x, y) or dist_to_start < height/10): #TODO: arbitrary
            print "generating goal"
            x = random.randint(0 + robot_radius, width - robot_radius)
            y = random.randint(0 + robot_radius, height - robot_radius)
            dist_to_start = dist = numpy.sqrt(numpy.square(x - start_x) + numpy.square(y - start_y))
    else: # check if in collision with obstacles
        # print ("checking start collisions")
        while CheckCollision(env_config, x, y):
            x = random.randint(0 + robot_radius, width - robot_radius)
            y = random.randint(0 + robot_radius, height - robot_radius)
    config.append(float(x))
    config.append(float(y))
    config.append(theta)
    return config


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
            print ("Error checking collision for shape type \"" + shape[0] + "\".  Expecting R, L, C, or A.  Exiting.")
            exit(0)

    return False


def CheckRobotRectangleCollision(shape, x, y):
    # Rectangle contain top left, top right, bottom right, and bottom left coordinates    # Line contains start and end coordinate
    global height
    global width
    global robot_radius
    
    # print ("checking rectangle collision")
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
                Ab = -(y3 - y4)
                Bb = x3 - x4
                Cb = -(Ab * x4 + Bb * y4)
                Db = Ab * robot_x + Bb * robot_y + Cb
                # print "Db:",Db
                if Db >= 0: # on or above line
                    return True

    # maybe do this instead? http://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/ 

    # next check if robot is within robot_radius of edge of rectangle
    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line

    dist_left_side = PointCloseToLineSegment(robot_x, -robot_y, x1, -y1, x4, -y4)
    dist_right_side = PointCloseToLineSegment(robot_x, -robot_y, x2, -y2, x3, -y3)
    dist_top_side = PointCloseToLineSegment(robot_x, -robot_y, x3, -y3, x4, -y4)
    dist_bottom_side = PointCloseToLineSegment(robot_x, -robot_y, x1, -y1, x2, -y2)

    # print "dist_left_side:",dist_left_side
    # print "dist_right_side:",dist_right_side
    # print "dist_top_side:",dist_top_side
    # print "dist_bottom_side:",dist_bottom_side

    if dist_left_side  or dist_right_side or dist_top_side or dist_bottom_side:
        return True

    return False

def DistPointToLine(point_x, point_y, line1_x, line1_y, line2_x, line2_y):
    numerator = abs((line2_y - line1_y) * point_x - (line2_x - line1_x) * point_y + line2_x * line1_y - line2_y * line1_x)
    denominator = numpy.sqrt(numpy.square(line2_x - line1_x) + numpy.square(line2_y - line1_y))
    dist = numerator / denominator

    return dist


def PointCloseToLineSegment(point_x, point_y, line1_x, line1_y, line2_x, line2_y):
    global robot_radius

    mid_x = abs(line2_x - line1_x) / 2 + min(line1_x, line2_x)
    mid_y = abs(line2_y - line1_y) / 2 + min(line1_y, line2_y)

    if line1_x < line2_x:
        left_x = line1_x
        left_y = line1_y
        right_x = line2_x
        right_y = line2_y
    else:
        left_x = line2_x
        left_y = line2_y
        right_x = line1_x
        right_y = line1_y

    theta = FindRelativeAngle(left_x, left_y, mid_x, mid_y, False)
    theta_rotated = -theta

    xl_rotated = math.cos(theta_rotated) * (left_x - mid_x) - math.sin(theta_rotated) * (left_y - mid_y) + mid_x
    yl_rotated = math.sin(theta_rotated) * (left_x - mid_x) + math.cos(theta_rotated) * (left_y - mid_y) + mid_y
    xr_rotated = math.cos(theta_rotated) * (right_x - mid_x) - math.sin(theta_rotated) * (right_y - mid_y) + mid_x
    yr_rotated = math.sin(theta_rotated) * (right_x - mid_x) + math.cos(theta_rotated) * (right_y - mid_y) + mid_y
    xp_rotated = math.cos(theta_rotated) * (point_x - mid_x) - math.sin(theta_rotated) * (point_y - mid_y) + mid_x
    yp_rotated = math.sin(theta_rotated) * (point_x - mid_x) + math.cos(theta_rotated) * (point_y - mid_y) + mid_y


    if xl_rotated < xp_rotated and xp_rotated < xr_rotated:
        if abs(yp_rotated - yl_rotated) < (robot_radius - 0.4):
            return True
    else:
        dist_left = numpy.sqrt(float(numpy.square(xp_rotated - xl_rotated) + numpy.square(yp_rotated - yl_rotated)))
        dist_right = numpy.sqrt(float(numpy.square(xp_rotated - xr_rotated) + numpy.square(yp_rotated - yr_rotated)))
        if min(dist_left, dist_right) < (robot_radius - 0.4):
            return True

    return False


# calculate angle from origin to point p
def FindRelativeAngle(o_x, o_y, p_x, p_y, should_round = True):

    # y axis is flipped
    origin_x = float(o_x)
    origin_y = float(o_y)
    point_x = float(p_x)
    point_y = float(p_y)

    if origin_y != point_y:
        relative_theta = math.atan(float(point_y - origin_y) / float(point_x - origin_x)) # radians
    else:
        if point_x > origin_x: # point is to right of origin
            relative_theta = 0
        else: # point is to left of origin
            relative_theta = math.pi

    # theta is between -pi/4 and pi/4.  need to change to be between 0 and 2pi
    if point_x < origin_x: # quadrant 2 and 3
        relative_theta = relative_theta + math.pi
    elif point_y < origin_y: # quadrant 4
        relative_theta = relative_theta + 2*math.pi

    # print "relative_theta:",relative_theta
    if should_round:
        return round(relative_theta, 2)
    else:
        return relative_theta


def CheckRobotLineCollision(shape, x, y):
    # Line contains start and end coordinate
    global height
    global width
    global robot_radius
    
    # print ("line collision")
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
                    # print ("robot eating arc")
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


if __name__ == "__main__":

    height = 1000
    width = 1000
    robot_radius = 50

    parser = argparse.ArgumentParser(description='script for testing planners')
    
    parser.add_argument('-p', '--planner', type=str, default = 'v',
                        help='The planner to run (v,prm,rrt,vprm,vrrt)')

    parser.add_argument('-v', '--visualize', default = False,
                        help='Enable visualization of graph (default is False')

    parser.add_argument('-d', '--domain', default = 'r',
                        help='Select which domain to use (r for random, easy, medium, or hard for difficult of random, or enter file name.  Default is random)')

    parser.add_argument('-t', '--trials', default = False,
                        help='Run 50 trials of specific file with each planner (250 trials total)')
    
    args = parser.parse_args()

    domain = args.domain
    
    if args.visualize == 'True':
        visualize = True
    else:
        visualize = False
    
    if args.trials == 'True':
        trials = True
    else:
        trials = False

    planning_env = PlanningEnvironment(width, height, robot_radius)
    planner = args.planner
    if args.planner == 'v':
        planner_name = 'v'
    elif args.planner == 'rrt':
        planner_name = 'rrt'
    elif args.planner == 'prm':
        planner_name = 'prm'
    elif args.planner == 'vrrt':
        planner_name = 'vrrt'
    elif args.planner == 'vprm':
        planner_name = 'vprm'
    else:
        print ('Unknown planner option: %s' % args.planner)
        exit(0)

    main(planner, planning_env, visualize, domain, planner_name, trials)