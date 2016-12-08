import argparse, numpy, openravepy, time, math, random
from RRTTree import RRTTree
import matplotlib.pyplot as pl

class VRRTPlanner(object):

	def __init__(self, planning_env, visualize, width, height, robot_radius):
		self.planning_env = planning_env
		self.visualize = visualize
		self.width = width
		self.height = height
		self.robot_radius = robot_radius
		self.select_visible = True
		self.num_extended_pass = 0
		self.num_extended_fail = 0


	def Plan(self, env_config, start_config, goal_config):
		self.start_config = start_config
		self.goal_config = goal_config
		self.VisibilityVertices = self.planning_env.GetVisibleVertices(env_config)

		start_coord = start_config[0]
		goal_coord = goal_config[0]	
		self.tree = RRTTree(start_coord)
		
		numVV = float(len(self.VisibilityVertices))

		GoalProb = 0.2
		VertexProb = 0

		epsilon = self.robot_radius

		if self.visualize:
			self.InitializePlot(start_coord)

		self.tree = RRTTree(start_coord)

		GoalFound = False
		timeout_limit = 5.0

		start_time = time.time()
		run_time = time.time() - start_time

		while not(GoalFound) and run_time < timeout_limit:
			numExpanded = len(self.tree.Nodes2D)
			if self.select_visible and numExpanded > numVV:
				VertexProb = 1 - (numVV/numExpanded)
			random_coord = self.GenerateRandomNode(GoalProb, VertexProb, goal_coord)
			nearest_node = self.tree.GetNearestNode(random_coord)
			CoordInCollision = self.planning_env.CheckPathCollision(env_config, nearest_node, random_coord)

			# If random coordinate is reachable, extend to it, else towards it
			if not(CoordInCollision):
				new_coord = random_coord
			else:
				new_coord = self.ExtendTowardsCoord(nearest_node, random_coord)
				InCollision = self.planning_env.CheckPathCollision(env_config, nearest_node, new_coord)
				if InCollision:
					self.num_extended_fail += 1
					continue
				else:
					self.num_extended_pass += 1

			self.tree.AddEdge(nearest_node, new_coord)
			if self.visualize:
				self.PlotEdge(nearest_node, new_coord)

			DistToGoal = self.ComputeDistance(goal_coord, new_coord)
			if goal_coord == new_coord:
				GoalFound = True
			elif (DistToGoal < epsilon):
				self.tree.AddEdge(new_coord, goal_coord)
				GoalFound = True

			run_time = time.time() - start_time

		#Check if RRT completed
		if GoalFound == False:	
			if_fail = 1
			path3D = []
			len_path = 0
			construct_time = 0
		else:

			find_path_time = time.time()
			if_fail = 0
			path2D = [goal_coord];
			while path2D[-1] != start_coord:
				Parent = self.tree.NodeParent[path2D[-1]]
				previous = Parent[0]
				path2D.append(previous)
			path2D.reverse();
			path3D, len_path = self.planning_env.Construct3DPath(path2D, start_config, goal_config)

			construct_time = time.time() - find_path_time


		num_nodes = len(self.tree.Nodes2D)
		Vertices = self.tree.Nodes2D
		Edges = self.tree.NodeParent

		return Vertices, Edges, path3D, construct_time, NodeStats, len_path, if_fail


	def GenerateRandomNode(self, GoalProb, VertexProb, goal_coord):
		RandProb = numpy.random.random_sample()

		if RandProb < GoalProb:
			new_coord = goal_coord
		elif GoalProb <= RandProb and RandProb <= (GoalProb + VertexProb):
			new_coord = self.GetRandVertex()
		else:
			new_coord = self.GetRandCoord()
		
		return new_coord

	def GetRandVertex(self):
		all_in = True

		for vertex in self.VisibilityVertices:
			if vertex not in self.tree.Nodes2D:
				all_in = False
				break

		if all_in:
			self.select_visible = False
			vertex = self.GetRandCoord()
		else:
			MaxIndex = len(self.VisibilityVertices) - 1
			newVertex = False
			while not(newVertex):
				RandIndex = random.randint(0, MaxIndex)
				vertex = self.VisibilityVertices[RandIndex]
				if vertex not in self.tree.Nodes2D:
					newVertex = True


		return vertex


	def GetPointAtDistOnLine(self, start, end, dist):
		start_x = start[0]
		start_y = start[1]
		end_x = end[0]
		end_y = end[1]

		vector = (end_x - start_x, end_y - start_y)
		vectorMagnitude = math.sqrt(numpy.square(end_x - start_x) + numpy.square(end_y - start_y))
		unitVector = (vector[0]/vectorMagnitude, vector[1]/vectorMagnitude)

		delta_x = unitVector[0] * dist
		delta_y = unitVector[1] * dist

		new_coord_x = round(start[0] + delta_x, 2)
		new_coord_y = round(start[1] + delta_y, 2)
		new_coord = (new_coord_x, new_coord_y)

		return new_coord


	def GetRandCoord(self):
		rand_x = round(numpy.random.random_sample() * self.width, 2)
		rand_y = round(numpy.random.random_sample() * self.height, 2)

		return (rand_x, rand_y)


	def ExtendTowardsCoord(self, nearest_node, random_coord):
		start_x = nearest_node[0]
		start_y = nearest_node[1]
		end_x = random_coord[0]
		end_y = random_coord[1]

		vector = (end_x - start_x, end_y - start_y)
		vectorMagnitude = math.sqrt(numpy.square(end_x - start_x) + numpy.square(end_y - start_y))
		unitVector = (vector[0]/vectorMagnitude, vector[1]/vectorMagnitude)

		delta_x = unitVector[0] * self.robot_radius
		delta_y = unitVector[1] * self.robot_radius

		new_coord_x = round(nearest_node[0] + delta_x, 2)
		new_coord_y = round(nearest_node[1] + delta_y, 2)
		new_coord = (new_coord_x, new_coord_y)

		return new_coord


	def ComputeDistance(self, node1, node2):
		dist = numpy.sqrt(numpy.square(node1[0]-node2[0])+numpy.square(node1[1]-node2[1]))
		return dist


	def InitializePlot(self, goal_config):
		self.fig = pl.figure()
		border = self.robot_radius/2
		lower_limits = [0-border, 0-border]
		upper_limits = [self.width+border, self.height+border]
		pl.xlim([lower_limits[0], upper_limits[0]])
		pl.ylim([lower_limits[1], upper_limits[1]])
		pl.plot(goal_config[0], goal_config[1], 'gx')
		pl.ion()
		pl.show()


	def PlotEdge(self, start, end):
		pl.plot([start[0], end[0]],
				[start[1], end[1]],
				'k.-', linewidth=2.5)
		pl.draw()







