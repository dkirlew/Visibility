import argparse, numpy, openravepy, time, math, random
from RRTTree import RRTTree
import matplotlib.pyplot as pl

class RRTPlanner(object):

	def __init__(self, planning_env, visualize, width, height, robot_radius):
		self.planning_env = planning_env
		self.visualize = visualize
		self.width = width
		self.height = height
		self.robot_radius = robot_radius


	def Plan(self, env_config, start_config, goal_config):

		self.env_config = env_config
		self.start_config = start_config
		self.goal_config = goal_config

		GoalProb = 0.2
		epsilon = self.robot_radius/2

		start_coord = start_config[0]
		goal_coord = goal_config[0]	

		if self.visualize:
			self.InitializePlot(start_coord)

		tree = RRTTree(start_coord)

		GoalFound = False

		while not(GoalFound):
			random_coord = self.GeneratePermissableRandomNode(GoalProb, goal_coord)
			# print "random_coord", random_coord
			shortest_dist, nearest_node = tree.GetNearestNode(random_coord)
			new_coord = self.Extend(nearest_node, random_coord)
			InCollision = self.NewEdgeCollision(nearest_node, new_coord)
			if not(InCollision):
				tree.AddEdge(nearest_node, new_coord)
				self.PlotEdge(nearest_node, new_coord)
				# print "new_coord", new_coord

				DistToGoal = self.ComputeDistance(goal_coord, new_coord)
				if (DistToGoal < epsilon):
					tree.AddEdge(new_coord, goal_coord)
					GoalFound = True

		print "GOAL FOUND"

		Plan = [goal_coord];
		while Plan[-1] != start_coord:
			previous = tree.NodeParent[Plan[-1]]
			Plan.append(previous)

		Plan.reverse();
		print Plan



		return Plan, Plan, Plan


	def GeneratePermissableRandomNode(self, GoalProb, goal_coord):

		RandProb = numpy.random.random_sample()

		if RandProb < GoalProb:
			return goal_coord
		else:
			new_coord = self.GetRandCoord()
			return new_coord


			# collision = True
			# while (collision):
			# 	rand_x = round(numpy.random.random_sample() * self.width, 2)
			# 	rand_y = round(numpy.random.random_sample() * self.height, 2)
			# 	collision = self.planning_env.CheckCollision(self.env_config, rand_x, rand_y)

			# 	if collision:
			# 		print "OH NO! COLLISION"
			# 	else:
			# 		new_coord = (rand_x, rand_y)
			# 		print "New point: ", new_coord

			# return new_coord

	def NewEdgeCollision(self, start, end):

		start_x = start[0]
		start_y = start[1]
		end_x = end[0]
		end_y = end[1]

		dist = numpy.sqrt(float(numpy.square(start_x - end_x) + numpy.square(start_y - end_y)))
		check_dist = 0.0
		while check_dist <= dist:
			temp_loc = self.GetPointAtDistOnLine(start, end, check_dist)
			if self.planning_env.CheckCollision(self.env_config, temp_loc[0], temp_loc[1]):
				print "ON NO! COLLISION"
				return True
			check_dist+=(dist / 5.0)
	
		if self.planning_env.CheckCollision(self.env_config, end_x, end_y):
			return True

		return False

	def GetPointAtDistOnLine(self, start, end, dist):

		start_x = start[0]
		start_y = start[1]
		end_x = end[0]
		end_y = end[1]

		vector = (end_x - start_x, end_y - start_y)
		vectorMagnitude = math.sqrt(numpy.square(end_x - start_x) + numpy.square(end_y - start_y))
		unitVector = (vector[0]/vectorMagnitude, vector[1]/vectorMagnitude)

		# delta = 0.2
		# dist = self.ComputeDistance(nearest_node, random_coord)
		# delta_dist = dist*delta
		delta_x = unitVector[0] * dist
		delta_y = unitVector[1] * dist

		new_coord_x = round(start[0] + delta_x, 2)
		new_coord_y = round(start[1] + delta_y, 2)
		new_coord = (new_coord_x, new_coord_y)
		# print "new_coord", new_coord

		return new_coord





	def GetRandCoord(self):
		rand_x = round(numpy.random.random_sample() * self.width, 2)
		rand_y = round(numpy.random.random_sample() * self.height, 2)

		return (rand_x, rand_y)



	def Extend(self, nearest_node, random_coord):

		start_x = nearest_node[0]
		start_y = nearest_node[1]
		end_x = random_coord[0]
		end_y = random_coord[1]

		vector = (end_x - start_x, end_y - start_y)
		vectorMagnitude = math.sqrt(numpy.square(end_x - start_x) + numpy.square(end_y - start_y))
		unitVector = (vector[0]/vectorMagnitude, vector[1]/vectorMagnitude)

		# delta = 0.2
		# dist = self.ComputeDistance(nearest_node, random_coord)
		# delta_dist = dist*delta
		delta_x = unitVector[0] * self.robot_radius
		delta_y = unitVector[1] * self.robot_radius

		new_coord_x = round(nearest_node[0] + delta_x, 2)
		new_coord_y = round(nearest_node[1] + delta_y, 2)
		new_coord = (new_coord_x, new_coord_y)
		# print "new_coord", new_coord

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







