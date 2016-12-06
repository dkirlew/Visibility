import numpy
import random, math
import time
import pygame

class PlanningEnvironment(object):

	def __init__(self, width, height, robot_radius):

		self.width = width
		self.height = height
		self.robot_radius = robot_radius

		self.BLACK  = (  0,   0,   0)
		self.WHITE  = (255, 255, 255)
		self.BLUE   = ( 51, 102, 255)
		self.GREEN  = (  0, 255,   0)
		self.RED    = (255,   0,   0)
		self.PURPLE = (255,   0, 255)
		self.ORANGE = (255,  69,   0)
		self.MAROON = (153,   0,  51)

		return

	def CheckCollision(self, env_config, x, y):
		
		for shape in env_config:
			if shape[0].lower() == "r":
				if self.CheckRobotRectangleCollision(shape, x, y):
					return True
			elif shape[0].lower() == "l":
				if self.CheckRobotLineCollision(shape, x, y):
					return True
			elif shape[0].lower() == "c":
				if self.CheckRobotCircleCollision(shape, x, y):
					return True
			else:
				print ("Error checking collision for shape type \"" + shape[0] + "\".  Expecting R, L, C, or A.  Exiting.")
				exit(0)

		return False


	def CheckRobotRectangleCollision(self, shape, x, y):
		# Rectangle contain top left, top right, bottom right, and bottom left coordinates    # Line contains start and end coordinate
		
		# print ("rectangle collision")
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

		dist_left_side = self.PointCloseToLineSegment(robot_x, robot_y, x1, y1, x4, y4)
		dist_right_side = self.PointCloseToLineSegment(robot_x, robot_y, x2, y2, x3, y3)
		dist_top_side = self.PointCloseToLineSegment(robot_x, robot_y, x3, y3, x4, y4)
		dist_bottom_side = self.PointCloseToLineSegment(robot_x, robot_y, x1, y1, x2, y2)

		# print "dist_left_side:",dist_left_side
		# print "dist_right_side:",dist_right_side
		# print "dist_top_side:",dist_top_side
		# print "dist_bottom_side:",dist_bottom_side

		if dist_left_side or dist_right_side or dist_top_side or dist_bottom_side :
			return True


		return False

	def DistPointToLine(self, point_x, point_y, line1_x, line1_y, line2_x, line2_y):
		numerator = abs((line2_y - line1_y) * point_x - (line2_x - line1_x) * point_y + line2_x * line1_y - line2_y * line1_x)
		denominator = numpy.sqrt(numpy.square(line2_x - line1_x) + numpy.square(line2_y - line1_y))
		dist = numerator / denominator

		return dist


	def PointCloseToLineSegment(self, point_x, point_y, line1_x, line1_y, line2_x, line2_y):
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

		theta = self.FindRelativeAngle(left_x, left_y, mid_x, mid_y, False)
		theta_rotated = -theta

		xl_rotated = math.cos(theta_rotated) * (left_x - mid_x) - math.sin(theta_rotated) * (left_y - mid_y) + mid_x
		yl_rotated = math.sin(theta_rotated) * (left_x - mid_x) + math.cos(theta_rotated) * (left_y - mid_y) + mid_y
		xr_rotated = math.cos(theta_rotated) * (right_x - mid_x) - math.sin(theta_rotated) * (right_y - mid_y) + mid_x
		yr_rotated = math.sin(theta_rotated) * (right_x - mid_x) + math.cos(theta_rotated) * (right_y - mid_y) + mid_y
		xp_rotated = math.cos(theta_rotated) * (point_x - mid_x) - math.sin(theta_rotated) * (point_y - mid_y) + mid_x
		yp_rotated = math.sin(theta_rotated) * (point_x - mid_x) + math.cos(theta_rotated) * (point_y - mid_y) + mid_y

		if xl_rotated < xp_rotated and xp_rotated < xr_rotated:
			if abs(yp_rotated - yl_rotated) < (self.robot_radius - 0.4):
				# print "in box"
				# print "xl_rotated",xl_rotated
				# print "yl_rotated",yl_rotated
				# print "xr_rotated",xr_rotated
				# print "yr_rotated",yr_rotated
				# print "xp_rotated",xp_rotated
				# print "yp_rotated",yp_rotated
				return True
		else:
			dist_left = numpy.sqrt(float(numpy.square(xp_rotated - xl_rotated) + numpy.square(yp_rotated - yl_rotated)))
			dist_right = numpy.sqrt(float(numpy.square(xp_rotated - xr_rotated) + numpy.square(yp_rotated - yr_rotated)))
			if min(dist_left, dist_right) < (self.robot_radius - 0.4):
				# print "in sides"
				# print "xl_rotated",xl_rotated
				# print "yl_rotated",yl_rotated
				# print "xr_rotated",xr_rotated
				# print "yr_rotated",yr_rotated
				# print "xp_rotated",xp_rotated
				# print "yp_rotated",yp_rotated
				return True

		return False


	# calculate angle from origin to point p
	def FindRelativeAngle(self, o_x, o_y, p_x, p_y, should_round = True):

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


	def CheckRobotLineCollision(self, shape, x, y):
		# Line contains start and end coordinate
		
		# print ("line collision")
		sx = shape[1][0] # start x
		sy = shape[1][1] # start y
		ex = shape[2][0] # end x
		ey = shape[2][1] # end y

		# # create rectangle by padding line with robot_radius.  Check for collision between new rectangle and robot
		# dist = self.DistPointToLine(x, y, sx, sy, ex, ey)

		# if dist < self.robot_radius: # inside bar around line (line extends between infinities)
		# 	dist_start = numpy.sqrt(numpy.square(sx - x) + numpy.square(sy - y))
		# 	dist_end = numpy.sqrt(numpy.square(ex - x) + numpy.square(ey - y))
		# 	if (x > min(sx, ex) and x < max(sx, ex)) or (y > min(sy, ey) and y < max(sy, ey)):
		# 		return True
		# 	elif dist_start < self.robot_radius or dist_end < self.robot_radius:
		# 		return True

		return self.PointCloseToLineSegment(x, y, sx, sy, ex, ey)


	def CheckRobotCircleCollision(self, shape, x, y):
		# Circle contains coordinate of center and radius
		
		cx = int(shape[1][0]) # center x
		cy = int(shape[1][1])# center y
		radius = shape[2] + self.robot_radius

		# check if distance from center to robot is less than radius of circle plus robot radius
		dist = numpy.sqrt(numpy.square(cx - x) + numpy.square(cy - y))
		if dist < radius:
			return True

		return False


	def InitializeMiniPlot(self, env_config, start_config, goal_config, name):
		# R = Rectangle, L = Line, C = Circle, A = Arc, S = Start Config, G = Goal Config
		# Rectangle contain top left, top right, bottom right, and bottom left coordinates
		# Line contains start and end coordinate
		# Circle contains coordinate of center and radius
		# Arc contains coordinate of center, width and height of rectangle containing the arc, start angle, and stop angle.  When added to env_config, Arc will also contain coordinate of center of subtended arc
		# Start config contains x and y coordinates of robot and theta of robot config
		# Goal config contains x and y coordinates of robot and theta of robot config

		border_size = 50

		pygame.init()
		self.screen = pygame.display.set_mode((self.width + 2 * border_size, self.height + 2 * border_size), pygame.DOUBLEBUF)
		self.background = pygame.Surface(self.screen.get_size()).convert()
		self.background.fill((255, 255, 255))
		# self.background = self.background.convert()
		# self.screen.blit(self.background, (0, 0))
		self.clock = pygame.time.Clock()
		self.fps = 30
		self.playtime = 0.0

		# draw border
		pygame.draw.rect(self.background, self.MAROON, [border_size, border_size, self.width, self.height], 5)

		start_robot_x = int(start_config[0][0]) + border_size # robot x
		start_robot_y = int(start_config[0][1]) + border_size # robot y
		start_theta = float(start_config[1]) # * 180 / math.pi

		# find where ID dot of robot should be to indicate orientation
		# http://math.libretexts.org/Core/Calculus/Precalculus/Chapter_5%3A_Trigonometric_Functions_of_Angles/5.3_Points_on_Circles_using_Sine_and_Cosine
		start_theta_x = int((self.robot_radius / 2) * math.cos(start_theta)) + start_robot_x
		start_theta_y = -1 * int((self.robot_radius / 2) * math.sin(start_theta)) + start_robot_y # inverting y because origin for pygame is at top left, origin in coordinate system is in bottom left


		pygame.draw.circle(self.background, self.GREEN, (start_robot_x, start_robot_y), self.robot_radius)
		pygame.draw.circle(self.background, self.BLACK, (start_theta_x, start_theta_y), self.robot_radius/4)

		goal_robot_x = int(goal_config[0][0]) + border_size # robot x
		goal_robot_y = int(goal_config[0][1]) + border_size # robot y
		goal_theta = float(goal_config[1])# * 180 / math.pi

		# find where ID dot of robot should be to indicate orientation
		# http://math.libretexts.org/Core/Calculus/Precalculus/Chapter_5%3A_Trigonometric_Functions_of_Angles/5.3_Points_on_Circles_using_Sine_and_Cosine
		goal_theta_x = int((self.robot_radius / 2) * math.cos(goal_theta)) + goal_robot_x
		goal_theta_y = -1 * int((self.robot_radius / 2) * math.sin(goal_theta)) + goal_robot_y # inverting y because origin for pygame is at top left, origin in coordinate system is in bottom left

		pygame.draw.circle(self.background, self.RED, (goal_robot_x, goal_robot_y), self.robot_radius)
		pygame.draw.circle(self.background, self.BLACK, (goal_theta_x, goal_theta_y), self.robot_radius/4)
				


		# Show all obstacles in environment
		for obstacle in env_config:
			if obstacle[0].lower() == "r":

				x1 = obstacle[1][0] + border_size # top left x
				y1 = obstacle[1][1] + border_size # top left y
				x2 = obstacle[2][0] + border_size # top right x
				y2 = obstacle[2][1] + border_size # top right y
				x3 = obstacle[3][0] + border_size # bottom right x
				y3 = obstacle[3][1] + border_size # bottom right y
				x4 = obstacle[4][0] + border_size # bottom left x
				y4 = obstacle[4][1] + border_size # bottom left y

				pygame.draw.polygon(self.background, self.GREEN, ((x1, y1), (x2, y2), (x3, y3), (x4, y4)))

			elif obstacle[0].lower() == "l":

				sx = obstacle[1][0] + border_size # start x
				sy = obstacle[1][1] + border_size # start y
				ex = obstacle[2][0] + border_size # end x
				ey = obstacle[2][1] + border_size # end y

				pygame.draw.line(self.background, self.BLACK, [sx, sy], [ex, ey])

			elif obstacle[0].lower() == "c":

				cx = int(obstacle[1][0]) + border_size # center x
				cy = int(obstacle[1][1]) + border_size # center y
				radius = int(obstacle[2])

				pygame.draw.circle(self.background, self.BLUE, (cx, cy), radius)

			elif obstacle[0].lower() == "a":

				x = int(obstacle[1][0]) + border_size # top left x
				y = int(obstacle[1][1]) + border_size # top left y
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

		pygame.image.save(self.background, "Images/" + name + "-" + str(time.time()) + ".jpg")


	def InitializePlot(self, Vertices, Edges, path, env_config, start_config, goal_config, file_name, planner_name, trial_num):
		# R = Rectangle, L = Line, C = Circle, A = Arc, S = Start Config, G = Goal Config
		# Rectangle contain top left, top right, bottom right, and bottom left coordinates
		# Line contains start and end coordinate
		# Circle contains coordinate of center and radius
		# Arc contains coordinate of center, width and height of rectangle containing the arc, start angle, and stop angle.  When added to env_config, Arc will also contain coordinate of center of subtended arc
		# Start config contains x and y coordinates of robot and theta of robot config
		# Goal config contains x and y coordinates of robot and theta of robot config

		border_size = 50

		pygame.init()
		self.screen = pygame.display.set_mode((self.width + border_size * 2, self.height + border_size * 2), pygame.DOUBLEBUF)
		self.background = pygame.Surface(self.screen.get_size()).convert()
		self.background.fill((255, 255, 255))
		# self.background = self.background.convert()
		# self.screen.blit(self.background, (0, 0))
		self.clock = pygame.time.Clock()
		self.fps = 30
		self.playtime = 0.0


		# draw border
		pygame.draw.rect(self.background, self.MAROON, [border_size, border_size, self.width, self.height], 5)

		# print "robot start config"
		start_robot_x = int(start_config[0][0]) + border_size # robot x
		start_robot_y = int(start_config[0][1]) + border_size # robot y
		start_theta = -1 * float(start_config[1]) # * 180 / math.pi
		# print "start_theta:",start_theta

		# find where ID dot of robot should be to indicate orientation
		# http://math.libretexts.org/Core/Calculus/Precalculus/Chapter_5%3A_Trigonometric_Functions_of_Angles/5.3_Points_on_Circles_using_Sine_and_Cosine
		start_theta_x = int((self.robot_radius / 2) * math.cos(start_theta)) + start_robot_x
		start_theta_y = -1 * int((self.robot_radius / 2) * math.sin(start_theta)) + start_robot_y # inverting y because origin for pygame is at top left, origin in coordinate system is in bottom left


		pygame.draw.circle(self.background, self.GREEN, (start_robot_x, start_robot_y), self.robot_radius)
		pygame.draw.circle(self.background, self.BLACK, (start_theta_x, start_theta_y), self.robot_radius/4)

		# print "robot goal config"
		goal_robot_x = int(goal_config[0][0]) + border_size # robot x
		goal_robot_y = int(goal_config[0][1]) + border_size # robot y
		goal_theta = -1 * float(goal_config[1]) # * 180 / math.pi
		# print "goal_theta:",goal_theta

		# find where ID dot of robot should be to indicate orientation
		# http://math.libretexts.org/Core/Calculus/Precalculus/Chapter_5%3A_Trigonometric_Functions_of_Angles/5.3_Points_on_Circles_using_Sine_and_Cosine
		goal_theta_x = int((self.robot_radius / 2) * math.cos(goal_theta)) + goal_robot_x
		goal_theta_y = -1 * int((self.robot_radius / 2) * math.sin(goal_theta)) + goal_robot_y # inverting y because origin for pygame is at top left, origin in coordinate system is in bottom left

		pygame.draw.circle(self.background, self.RED, (goal_robot_x, goal_robot_y), self.robot_radius)
		pygame.draw.circle(self.background, self.BLACK, (goal_theta_x, goal_theta_y), self.robot_radius/4)
				


		# Show all obstacles in environment
		for obstacle in env_config:
			if obstacle[0].lower() == "r":
				# print "rectangle"

				x1 = obstacle[1][0] + border_size # top left x
				y1 = obstacle[1][1] + border_size # top left y
				x2 = obstacle[2][0] + border_size # top right x
				y2 = obstacle[2][1] + border_size # top right y
				x3 = obstacle[3][0] + border_size # bottom right x
				y3 = obstacle[3][1] + border_size # bottom right y
				x4 = obstacle[4][0] + border_size # bottom left x
				y4 = obstacle[4][1] + border_size # bottom left y

				pygame.draw.polygon(self.background, self.GREEN, ((x1, y1), (x2, y2), (x3, y3), (x4, y4)))

			elif obstacle[0].lower() == "l":
				# print "line"

				sx = obstacle[1][0] + border_size # start x
				sy = obstacle[1][1] + border_size # start y
				ex = obstacle[2][0] + border_size # end x
				ey = obstacle[2][1] + border_size # end y

				pygame.draw.line(self.background, self.BLACK, [sx, sy], [ex, ey], 4)

			elif obstacle[0].lower() == "c":
				# print "circle"

				cx = int(obstacle[1][0]) + border_size # center x
				cy = int(obstacle[1][1]) + border_size # center y
				radius = int(obstacle[2])

				pygame.draw.circle(self.background, self.BLUE, (cx, cy), radius)

			elif obstacle[0].lower() == "a":
				# print "arc"

				x = int(obstacle[1][0]) + border_size # top left x
				y = int(obstacle[1][1]) + border_size # top left y
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
				

		vertex_radius = 10
		for node in Vertices:#.items():
			# print "node",node
			node_x = int(node[0]) + border_size
			node_y = int(node[1]) + border_size

			pygame.draw.circle(self.background, self.BLACK, (node_x, node_y), vertex_radius)

		for edge, neighbors in Edges.items():
			edge_x = float(edge[0]) + border_size
			edge_y = float(edge[1]) + border_size
			for neighbor in neighbors:
				neighbor_x = float(neighbor[0]) + border_size
				neighbor_y = float(neighbor[1]) + border_size
				pygame.draw.line(self.background, self.ORANGE, [edge_x, edge_y], [neighbor_x, neighbor_y], 5)

		prev_state = None

		for state in path:
			state_x = state[0][0] + border_size
			state_y = state[0][1] + border_size
			state_theta = -1 * state[1]


			if prev_state!= None:
				prev_state_x = prev_state[0][0] + border_size
				prev_state_y = prev_state[0][1] + border_size

				# line of path being followed
				pygame.draw.line(self.background, self.BLACK, [state_x, state_y], [prev_state_x, prev_state_y])

			# print "state:",state_x,state_y,state_theta
			theta_x = int((self.robot_radius / 2) * math.cos(state_theta)) + state_x
			theta_y = -1 * int((self.robot_radius / 2) * math.sin(state_theta)) + state_y # inverting y because origin for pygame is at top left, origin in coordinate system is in bottom left

			pygame.draw.circle(self.background, self.PURPLE, (int(state_x), int(state_y)), self.robot_radius, 3)
			pygame.draw.circle(self.background, self.BLACK, (int(theta_x), int(theta_y)), self.robot_radius/3, 1)

			prev_state = state
		

		running = True
		while running:
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					running = False 
				elif event.type == pygame.KEYDOWN:
					if event.key == pygame.K_ESCAPE:
						running = False
					elif event.key == pygame.K_SPACE:
						running = False

			milliseconds = self.clock.tick(self.fps)
			self.playtime += milliseconds / 1000.0
			# self.draw_text("FPS: {:6.3}{}PLAYTIME: {:6.3} SECONDS".format(
						   # self.clock.get_fps(), " "*5, self.playtime))

			pygame.display.flip()
			self.screen.blit(self.background, (0, 0))

		pygame.image.save(self.background, planner_name + "-" + file_name + "-" + str(trial_num) + ".jpg")
		

	def Construct3DPath(self, path2D, start_config, goal_config):
		path3D = [start_config]
		cost = 0.0

		prev_node = (start_config[0][0], start_config[0][1])

		for i in range(1, len(path2D)):
			prev_node = path2D[i-1]
			# print "prev_node:",prev_node
			prev_node_x = prev_node[0]
			prev_node_y = prev_node[1]
			node = path2D[i]
			# print "node:",node
			node_x = node[0]
			node_y = node[1]

			theta = self.FindRelativeAngle(prev_node_x, prev_node_y, node_x, node_y, True)
			# print "theta:",theta

			path3D.append((prev_node, theta)) 	# incoming angle rotating
			path3D.append((node, theta)) 		# incoming movement

		path3D.append(goal_config)

		for i in range(1, len(path3D)):
			prev_node = path3D[i-1]
			node = path3D[i]

			cost+=self.CostOfMove(prev_node, node)

		return path3D, cost


	def FindRelativeAngle(self, o_x, o_y, p_x, p_y, should_round = True):

		origin_x = float(o_x)
		origin_y = float(o_y)
		point_x = float(p_x)
		point_y = float(p_y)

		if origin_x != point_x:
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

		if should_round:
			return round(relative_theta, 2)
		else:
			return relative_theta


	def CheckPathCollision(self, env_config, vertex, neighbor):
		# print "vertex:",vertex
		# print "neighbor:",neighbor

		vertex_x = vertex[0]
		vertex_y = vertex[1]
		neighbor_x = neighbor[0]
		neighbor_y = neighbor[1]

		dist = numpy.sqrt(float(numpy.square(vertex_x - neighbor_x) + numpy.square(vertex_y - neighbor_y)))
		check_dist = 0.0
		while check_dist <= dist:
			temp_loc = self.GetPointAtDistOnLine(vertex, neighbor, check_dist)
			if self.CheckCollision(env_config, temp_loc[0], temp_loc[1]):
				return True
			check_dist+=(self.robot_radius / 2.0)
		
		if self.CheckCollision(env_config, neighbor_x, neighbor_y):
				return True

		return False


	def GetPointAtDistOnLine(self, start_loc, end_loc, dist):
		start_x = float(start_loc[0])
		start_y = float(start_loc[1])
		end_x = float(end_loc[0])
		end_y = float(end_loc[1])

		point = [start_x, start_y]

		vector = [end_x - start_x, end_y - start_y]
		vectorMagnitude = math.sqrt(numpy.square(end_x - start_x) + numpy.square(end_y - start_y))
		unitVector = [vector[0]/vectorMagnitude, vector[1]/vectorMagnitude]

		point[0]+=(dist * unitVector[0])
		point[1]+=(dist * unitVector[1])

		return point

	def GetVisibleVertices(self, env_config):
		Vertices = []

		for shape in env_config:
			if shape[0] == "R":
				#rectangle
				self.GetRectangleVertices(env_config, shape, Vertices)

			elif shape[0] == "L":
				#line
				self.GetLineVertices(env_config, shape, Vertices)

			elif shape[0] == "C":
				#circle
				self.GetCircleVertices(env_config, shape, Vertices)
			else:
				print ("Shape not recognized. (Should be R or L or C or A)")
				exit(0)

		# dict is of form:
		#	key = 2D vertex
		#	val = list of adjacent 2D vertices
		return Vertices


	def GetRectangleVertices(self, env_config, shape, RectangleVertices):
		#http://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point
		
		x1 = float(shape[1][0]) # top left x
		y1 = float(shape[1][1]) # top left y
		x2 = float(shape[2][0]) # top right x
		y2 = float(shape[2][1]) # top right y
		x3 = float(shape[3][0]) # bottom right x
		y3 = float(shape[3][1]) # bottom right y
		x4 = float(shape[4][0]) # bottom left x
		y4 = float(shape[4][1]) # bottom left y


		# print "x1:",x1,"y1:",y1
		# print "x2:",x2,"y2:",y2
		# print "x3:",x3,"y3:",y3
		# print "x4:",x4,"y4:",y4

		xm = abs(x1 - x3) / 2 + min(x1, x3)
		ym = abs(y1 - y3) / 2 + min(y1, y3)

		x_side = abs(x1 - x2) / 2 + min(x1, x2)
		y_side = abs(y1 - y2) / 2 + min(y1, y2)

		# print "xm:",xm,"ym:",ym

		theta = self.FindRelativeAngle(x_side, y_side, xm, ym, False)
		theta_rotated = -theta

		# print "theta:",theta


		x1_rotated = round(math.cos(theta_rotated) * (x1 - xm) - math.sin(theta_rotated) * (y1 - ym) + xm, 2)
		y1_rotated = round(math.sin(theta_rotated) * (x1 - xm) + math.cos(theta_rotated) * (y1 - ym) + ym, 2)
		x2_rotated = round(math.cos(theta_rotated) * (x2 - xm) - math.sin(theta_rotated) * (y2 - ym) + xm, 2)
		y2_rotated = round(math.sin(theta_rotated) * (x2 - xm) + math.cos(theta_rotated) * (y2 - ym) + ym, 2)
		x3_rotated = round(math.cos(theta_rotated) * (x3 - xm) - math.sin(theta_rotated) * (y3 - ym) + xm, 2)
		y3_rotated = round(math.sin(theta_rotated) * (x3 - xm) + math.cos(theta_rotated) * (y3 - ym) + ym, 2)
		x4_rotated = round(math.cos(theta_rotated) * (x4 - xm) - math.sin(theta_rotated) * (y4 - ym) + xm, 2)
		y4_rotated = round(math.sin(theta_rotated) * (x4 - xm) + math.cos(theta_rotated) * (y4 - ym) + ym, 2)



		# print "x1_rotated:",x1_rotated,"y1_rotated:",y1_rotated
		# print "x2_rotated:",x2_rotated,"y2_rotated:",y2_rotated
		# print "x3_rotated:",x3_rotated,"y3_rotated:",y3_rotated
		# print "x4_rotated:",x4_rotated,"y4_rotated:",y4_rotated


		buffer_x1_rotated = x1_rotated - self.robot_radius
		buffer_y1_rotated = y1_rotated + self.robot_radius
		buffer_x2_rotated = x2_rotated - self.robot_radius
		buffer_y2_rotated = y2_rotated - self.robot_radius
		buffer_x3_rotated = x3_rotated + self.robot_radius
		buffer_y3_rotated = y3_rotated - self.robot_radius
		buffer_x4_rotated = x4_rotated + self.robot_radius
		buffer_y4_rotated = y4_rotated + self.robot_radius

		# print "buffer_x1_rotated:",buffer_x1_rotated,"buffer_y1_rotated:",buffer_y1_rotated
		# print "buffer_x2_rotated:",buffer_x2_rotated,"buffer_y2_rotated:",buffer_y2_rotated
		# print "buffer_x3_rotated:",buffer_x3_rotated,"buffer_y3_rotated:",buffer_y3_rotated
		# print "buffer_x4_rotated:",buffer_x4_rotated,"buffer_y4_rotated:",buffer_y4_rotated



		new_x1 = round(math.cos(theta) * (buffer_x1_rotated - xm) - math.sin(theta) * (buffer_y1_rotated - ym) + xm, 2)
		new_y1 = round(math.sin(theta) * (buffer_x1_rotated - xm) + math.cos(theta) * (buffer_y1_rotated - ym) + ym, 2)
		new_x2 = round(math.cos(theta) * (buffer_x2_rotated - xm) - math.sin(theta) * (buffer_y2_rotated - ym) + xm, 2)
		new_y2 = round(math.sin(theta) * (buffer_x2_rotated - xm) + math.cos(theta) * (buffer_y2_rotated - ym) + ym, 2)
		new_x3 = round(math.cos(theta) * (buffer_x3_rotated - xm) - math.sin(theta) * (buffer_y3_rotated - ym) + xm, 2)
		new_y3 = round(math.sin(theta) * (buffer_x3_rotated - xm) + math.cos(theta) * (buffer_y3_rotated - ym) + ym, 2)
		new_x4 = round(math.cos(theta) * (buffer_x4_rotated - xm) - math.sin(theta) * (buffer_y4_rotated - ym) + xm, 2)
		new_y4 = round(math.sin(theta) * (buffer_x4_rotated - xm) + math.cos(theta) * (buffer_y4_rotated - ym) + ym, 2)

		# print "new_x1:",new_x1,"new_y1:",new_y1
		# print "new_x2:",new_x2,"new_y2:",new_y2
		# print "new_x3:",new_x3,"new_y3:",new_y3
		# print "new_x4:",new_x4,"new_y4:",new_y4



		if not self.CheckCollision(env_config, new_x1, new_y1):
			RectangleVertices.append((new_x1, new_y1))
		if not self.CheckCollision(env_config, new_x2, new_y2):
			RectangleVertices.append((new_x2, new_y2))
		if not self.CheckCollision(env_config, new_x3, new_y3):
			RectangleVertices.append((new_x3, new_y3))
		if not self.CheckCollision(env_config, new_x4, new_y4):
			RectangleVertices.append((new_x4, new_y4))

		return


	def GetLineVertices(self, env_config, shape, LineVertices):
		
		if float(shape[1][0]) < float(shape[2][0]):
			xs = float(shape[1][0]) # top left x
			ys = float(shape[1][1]) # top left y
			xe = float(shape[2][0]) # top right x
			ye = float(shape[2][1]) # top right
		elif float(shape[1][0]) > float(shape[2][0]):
			xe = float(shape[1][0]) # top left x
			ye = float(shape[1][1]) # top left y
			xs = float(shape[2][0]) # top right x
			ys = float(shape[2][1]) # top right
		else:
			if float(shape[1][1]) < float(shape[2][1]):
				xs = float(shape[1][0]) # top left x
				ys = float(shape[1][1]) # top left y
				xe = float(shape[2][0]) # top right x
				ye = float(shape[2][1]) # top right
			else:
				xe = float(shape[1][0]) # top left x
				ye = float(shape[1][1]) # top left y
				xs = float(shape[2][0]) # top right x
				ys = float(shape[2][1]) # top right

		# print "xs:",xs,"ys:",ys
		# print "xe:",xe,"ye:",ye

		theta = self.FindRelativeAngle(xs, ys, xe, ye, False)

		# print "theta:",theta

		# if theta > math.pi/2:
		# 	theta_rotated = math.pi - theta
		# elif theta < math.pi/2:
		theta_rotated = -theta

		xm = abs(xe-xs)/2 + min(xs, xe)
		ym = abs(ye-ys)/2 + min(ys, ye)

		xs_rotated = math.cos(theta_rotated) * (xs - xm) - math.sin(theta_rotated) * (ys - ym) + xm
		ys_rotated = math.sin(theta_rotated) * (xs - xm) + math.cos(theta_rotated) * (ys - ym) + ym
		xe_rotated = math.cos(theta_rotated) * (xe - xm) - math.sin(theta_rotated) * (ye - ym) + xm
		ye_rotated = math.sin(theta_rotated) * (xe - xm) + math.cos(theta_rotated) * (ye - ym) + ym
		# print "xs_rotated:",xs_rotated,"ys_rotated:",ys_rotated
		# print "xe_rotated:",xe_rotated,"ye_rotated:",ye_rotated

		x1 = xs_rotated - self.robot_radius
		y1 = ys_rotated - self.robot_radius
		x2 = xe_rotated + self.robot_radius
		y2 = ye_rotated - self.robot_radius
		x3 = xe_rotated + self.robot_radius
		y3 = ye_rotated + self.robot_radius
		x4 = xs_rotated - self.robot_radius
		y4 = ys_rotated + self.robot_radius


		# print "x1:",x1,"y1:",y1
		# print "x2:",x2,"y2:",y2
		# print "x3:",x3,"y3:",y3
		# print "x4:",x4,"y4:",y4



		x1_rotated = round(math.cos(theta) * (x1 - xm) - math.sin(theta) * (y1 - ym) + xm, 2)
		y1_rotated = round(math.sin(theta) * (x1 - xm) + math.cos(theta) * (y1 - ym) + ym, 2)
		x2_rotated = round(math.cos(theta) * (x2 - xm) - math.sin(theta) * (y2 - ym) + xm, 2)
		y2_rotated = round(math.sin(theta) * (x2 - xm) + math.cos(theta) * (y2 - ym) + ym, 2)
		x3_rotated = round(math.cos(theta) * (x3 - xm) - math.sin(theta) * (y3 - ym) + xm, 2)
		y3_rotated = round(math.sin(theta) * (x3 - xm) + math.cos(theta) * (y3 - ym) + ym, 2)
		x4_rotated = round(math.cos(theta) * (x4 - xm) - math.sin(theta) * (y4 - ym) + xm, 2)
		y4_rotated = round(math.sin(theta) * (x4 - xm) + math.cos(theta) * (y4 - ym) + ym, 2)


		# print "x1_rotated:",x1_rotated,"y1_rotated:",y1_rotated
		# print "x2_rotated:",x2_rotated,"y2_rotated:",y2_rotated
		# print "x3_rotated:",x3_rotated,"y3_rotated:",y3_rotated
		# print "x4_rotated:",x4_rotated,"y4_rotated:",y4_rotated

		# print "line (x1_rotated, y1_rotated):",(x1_rotated, y1_rotated)
		# print "line (x2_rotated, y2_rotated):",(x2_rotated, y2_rotated)
		# print "line (x3_rotated, y3_rotated):",(x3_rotated, y3_rotated)
		# print "line (x4_rotated, y4_rotated):",(x4_rotated, y4_rotated)


		if not self.CheckCollision(env_config, x1_rotated, y1_rotated):
			LineVertices.append((x1_rotated, y1_rotated))
		if not self.CheckCollision(env_config, x2_rotated, y2_rotated):
			LineVertices.append((x2_rotated, y2_rotated))
		if not self.CheckCollision(env_config, x3_rotated, y3_rotated):
			LineVertices.append((x3_rotated, y3_rotated))
		if not self.CheckCollision(env_config, x4_rotated, y4_rotated):
			LineVertices.append((x4_rotated, y4_rotated))

		return


	def GetCircleVertices(self, env_config, shape, CircleVertices):
		
		xc = float(shape[1][0]) 
		yc = float(shape[1][1])
		circle_radius = float(shape[2])

		buffer_dist = circle_radius + self.robot_radius

		x_start = xc + buffer_dist/math.cos(math.pi/8)
		y_start = yc

		x_curr = x_start
		y_curr = y_start

		for i in range(1, 8):
			theta = math.pi/4 * i
			x_temp = math.cos(theta) * (x_curr - xc) - math.sin(theta) * (y_curr - yc) + xc
			y_temp = math.sin(theta) * (x_curr - xc) + math.cos(theta) * (y_curr - yc) + yc
			# print "circle loop (x_temp, y_temp):",(x_temp, y_temp)
			# print "circle loop (x_curr, y_curr):",(x_curr, y_curr)

			if (x_temp, y_temp) not in CircleVertices:
				if not self.CheckCollision(env_config, x_temp, y_temp):
					CircleVertices.append((x_temp, y_temp))
			if (x_curr, y_curr) not in CircleVertices:
				if not self.CheckCollision(env_config, x_curr, y_curr):
					CircleVertices.append((x_curr, y_curr))
			x_curr = x_temp
			y_curr = y_temp

		# print "circle (x_curr, y_curr):",(x_curr, y_curr)
		# print "circle (x_start, y_start):",(x_start, y_start)
		if (x_curr, y_curr) not in CircleVertices:
			if not self.CheckCollision(env_config, x_curr, y_curr):
				CircleVertices.append((x_curr, y_curr))
		if (x_start, y_start) not in CircleVertices:
			if not self.CheckCollision(env_config, x_start, y_start):
				CircleVertices.append((x_start, y_start))

		return


	def CostOfMove(self, edge, neighbor):
		cost = 0

		edge_x = edge[0][0]
		edge_y = edge[0][1]
		edge_theta = edge[1]
		neighbor_x = neighbor[0][0]
		neighbor_y = neighbor[0][1]
		neighbor_theta = neighbor[1]

		# euclidian distance, scaled by longest possible distance to travel - diagonal
		if edge_theta == neighbor_theta:
			cost = numpy.sqrt(float(numpy.square(edge_x - neighbor_x) + numpy.square(edge_y - neighbor_y)))
			# cost/= numpy.sqrt(float(numpy.square(self.height) + numpy.square(self.width)))
		# rotational distance in radians
		else:
			cost = abs(edge_theta - neighbor_theta)
			if cost > math.pi: # take the shorter route
				cost = 2 * math.pi - cost
			cost/= math.pi
			
		return cost