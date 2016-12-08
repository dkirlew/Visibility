#!/usr/bin/env python

import argparse, numpy, openravepy, time, math, random, ast

from collections import defaultdict
from heapq import heappop, heappush

class PRMPlanner(object):

	def __init__(self, planning_env, visualize, width, height, robot_radius):
		self.planning_env = planning_env
		self.visualize = visualize
		self.width = width
		self.height = height
		self.robot_radius = robot_radius
		self.max_neighbors = 20
		self.max_dist = numpy.sqrt(numpy.square(width) + numpy.square(height)) / 2 #TODO - arbitrary
		self.bounce_walks = {}
		self.max_nodes = (self.width / (2 * self.robot_radius)) * (self.height / (2 * self.robot_radius))


	def Plan(self, env_config, start_config, goal_config):
		self.env_config = env_config
		self.start_config = start_config
		self.goal_config = goal_config

		N = []	
		E = {}

		start_time = time.time()

		N, E, learn_time, construct_time, expand_time = self.Learn(N, E)

		P, query_time = self.Query(N, E)

		total_time = time.time() - start_time

		if P == None:
			print "Error: graph is incomplete.  Exiting."
			path = []
			len_path = 0
			if_fail = 1
		else:
			path, len_path = self.planning_env.Construct3DPath(P, self.start_config, self.goal_config)
			# print "N:",N
			# print "E:",E
			# print "path3D:",path
			if_fail = 0

		num_nodes = len(N)

		return N, E, path, [total_time, learn_time, query_time, construct_time, expand_time], num_nodes, len_path, if_fail


	def Learn(self, N, E):
		start_time = time.time()
		N, E, construct_time = self.Construct(N, E)

		N, E, expand_time = self.Expand(N, E)

		learn_time = time.time() - start_time

		return N, E, learn_time, construct_time, expand_time


	def Construct(self, N, E):
		# approximate number of robots that could fit in C space
		num_nodes = 0
		start_time = time.time()

		while num_nodes < self.max_nodes:
			c = self.RandomLocation()

			# neighbors is a priority queue of format (distance to c, (x, y))
			# neighbors are all visible/collision free from c to neighbor
			neighbors = self.GetCloseNodes(c, N)
			# print "got close nodes"
			num_neigbhors = 0

			# print "neighbors:",neighbors

			if len(neighbors) != 0:
				N.append(c)
				while neighbors and num_neigbhors < self.max_neighbors:
					# print "checking neighbors"
					dist, neighbor = heappop(neighbors)

					# avoid cycles and collisions
					if not self.IsCyclic(c, neighbor, E) and not self.planning_env.CheckPathCollision(self.env_config, c, neighbor):
						# print "Adding c:",c,"and neighbor:",neighbor,"in Construct"
						if c in E:
							E[c].append(neighbor)
						else:
							E[c] = [neighbor]
							
						if neighbor in E:
							E[neighbor].append(c)
						else:
							E[neighbor] = [c]
					num_neigbhors+=1
				num_nodes+=1
			else:
				N.append(c)
		query_time = time.time() - start_time

		return N, E, query_time


	def RandomLocation(self):
		x = round(random.uniform(0, self.width), 2)
		y = round(random.uniform(0, self.height), 2)


		while self.planning_env.CheckCollision(self.env_config, x, y):
			x = round(random.uniform(0, self.width), 2)
			y = round(random.uniform(0, self.height), 2)

		location = (x, y)

		return location


	def GetCloseNodes(self, c, N):
		close_nodes = []
		c_x = c[0]
		c_y = c[1]
		for node in N:
			node_x = node[0]
			node_y = node[1]
			dist = numpy.sqrt(numpy.square(c_x - node_x) + numpy.square(c_y - node_y))
			if dist <= self.max_dist:
				heappush(close_nodes, (dist, node))

		return close_nodes


	def IsCyclic(self, c, neighbor, E):
		Edges = E # do not directly edit E
		if c not in Edges:
			return False

		nodes_to_check = [c]
		checked_nodes = [c]

		while nodes_to_check:
			# get connections from first node to check, then remove
			new_nodes = Edges[nodes_to_check[0]]
			del nodes_to_check[0]
			for new_node in new_nodes:
				if new_node == neighbor:
					return True

				if new_node not in checked_nodes:
					nodes_to_check.append(new_node)
					checked_nodes.append(new_node)

		return False


	def Expand(self, N, E):
		start_time = time.time()
		weighted_nodes, max_edges = self.GetWeightedNodes(N, E)
		num_expansions = 0
		max_expansions = max_edges / 2

		# print "weighted_nodes:",weighted_nodes
		# print "max_edges:",max_edges

		while weighted_nodes and num_expansions < max_expansions:
			c = self.ChooseNode(weighted_nodes)
			# print "expanding c:",c
			N, E = self.ExpandNode(c, N, E, max_edges)

			num_expansions+=1

		expand_time = time.time() - start_time

		return N, E, expand_time


	def GetWeightedNodes(self, N, E):
		unweighted_nodes = {}
		weighted_nodes = {}
		normalized_nodes = []
		total_edges = 0.0
		max_edges = 0
		weight_sum = 0.0

		for node in N:
			if node in E:
				num_edges = len(E[node])
			else:
				num_edges = 0
			unweighted_nodes[node] = num_edges
			total_edges+=num_edges
			if num_edges > max_edges:
				max_edges = num_edges
		# print "unweighted_nodes:",unweighted_nodes

		for node in unweighted_nodes:
			num_edges = unweighted_nodes[node]
			weighted_val = 1 - float(num_edges) / max_edges # invert to give higher weight to nodes with low num edges
			# print "node:",node
			# print "num_edges:",num_edges
			# print "un normalized weighted_val:",weighted_val
			weighted_nodes[node] = weighted_val
			weight_sum+=weighted_val

		# print "weighted_nodes:",weighted_nodes

		for node in weighted_nodes:
			weight = weighted_nodes[node]
			normalized_weight = float(weight) / weight_sum
			# print "weight:",weight
			# print "normalized_weight:",normalized_weight

			heappush(normalized_nodes, (normalized_weight, node))

		return normalized_nodes, max_edges


	def ChooseNode(self, nodes):
		weighted_nodes = [x[:] for x in nodes] # do not change 
		prob = random.uniform(0, 1)
		nodes = weighted_nodes
		cumulative_weight = 0

		# print "weighted_nodes:",weighted_nodes
		while nodes:
			weight, node = weighted_nodes.pop()
			if (weight + cumulative_weight) >= prob:
				return node
			cumulative_weight+=weight

		return


	def ExpandNode(self, c, N, E, max_expansions):
		num_expansions = 0 # TODO
		num_tries = 0 # TODO
		max_tries = 50 # TODO

		while num_tries < max_tries:
			if num_expansions < max_expansions:
				neighbor = self.RandomCloseLocation(c, self.max_dist / 2) # TODO
				# avoid cycles and collisions
				if not self.IsCyclic(c, neighbor, E) and not self.planning_env.CheckPathCollision(self.env_config, c, neighbor):
					if neighbor not in N:
						N.append(neighbor)
						# print "Expanding c:",c,"to neighbor:",neighbor,"in ExpandNode"
						if c in E:
							E[c].append(neighbor)
						else:
							E[c] = [neighbor]
							
						if neighbor in E:
							E[neighbor].append(c)
						else:
							E[neighbor] = [c]

				num_expansions+=1

			num_tries+=1


		return N, E 


	def RandomCloseLocation(self, c, dist):
		c_x = c[0]
		c_y = c[1]

		min_x = max(0, c_x - dist)
		max_x = min(self.width, c_x + dist)

		min_y = max(0, c_y - dist)
		max_y = min(self.height, c_y + dist)


		x = round(random.uniform(min_x, max_x), 2)
		y = round(random.uniform(min_y, max_y), 2)


		while self.planning_env.CheckCollision(self.env_config, x, y):
			x = round(random.uniform(min_x, max_x), 2)
			y = round(random.uniform(min_y, max_y), 2)

		location = (x, y)

		return location


	def Query(self, N, E):
		start_time = time.time()

		P_start = None
		P_goal = None

		# may not find close node or may not be able to bounce walk to Edges
		while P_start == None:
			config = (self.start_config[0][0], self.start_config[0][1])
			P_start = self.FindConnection(config, N, E)
			# print "found P_start node:",P_start

		while P_goal == None:
			config = (self.goal_config[0][0], self.goal_config[0][1])
			P_goal = self.FindConnection(config, N, E)
			# print "found P_goal node:",P_goal
		
		P = self.ConnectEntirePaths(P_start, P_goal, N, E)

		query_time = time.time() - start_time

		return P, query_time

	def FindConnection(self, c, N, E):
		close_nodes = self.GetCloseNodes(c, E) #use E because need close nodes to be connected
		# print "close_nodes:",close_nodes

		while close_nodes:
			dist, node = heappop(close_nodes)
			# print "checking node:",node

			if not self.planning_env.CheckPathCollision(self.env_config, c, node):
				return [node]

		connection_path = self.BounceWalk(c, N, E)

		return connection_path


	def BounceWalk(self, c, N, E):
		walk_path = [c]

		c_x = c[0]
		c_y = c[1]
		curr_x = c_x
		curr_y = c_y
		step_dist = self.robot_radius
		num_bounces = 0

		theta = round(random.uniform(0, 2 * math.pi), 2)
		delta_x = round(step_dist * math.cos(theta), 2)
		delta_y = round(step_dist * math.sin(theta), 2)

		while num_bounces < 5: # TODO

			if not self.planning_env.CheckPathCollision(self.env_config, (curr_x, curr_y), (delta_x + curr_x, delta_y + curr_y)):
				curr_x+=delta_x
				curr_y+=delta_y

				walk_path.append((curr_x, curr_y))
			else:
				theta = round(random.uniform(0, 2 * math.pi), 2)
				delta_x = round(step_dist * math.cos(theta), 2)
				delta_y = round(step_dist * math.sin(theta), 2)

				num_bounces+=1

		if walk_path != c:
			self.bounce_walks[c, (curr_x, curr_y)] = walk_path
			if (curr_x, curr_y) not in N:
				N.append((curr_x, curr_y))
				# print "Adding c:",c,"and neighbor:",neighbor,"in BounceWalk"
				if c in E:
					E[c].append((curr_x, curr_y))
				else:
					E[c] = [(curr_x, curr_y)]

				if (curr_x, curr_y) in E:
					E[(curr_x, curr_y)].append(c)
				else:
					E[(curr_x, curr_y)] = [c]

			return walk_path
		else:
			return None


	def ConnectEntirePaths(self, P_start, P_goal, N, E):
		path = [(self.start_config[0][0], self.start_config[0][1])]
		final_cost = 0
		last_element = None
		for element in P_start:
			path.append(element)
			last_element = element

		# print "path after p_start:",path

		start = path[len(path) - 1]
		goal = P_goal[0] # P_goal goes from goal to connecting node

		temp_path = self.VisibilityDijkstras(N, E, start, goal)

		if last_element not in temp_path:
			return None

		for element in temp_path:
			if element != start and element != goal:
				path.append(element)

		# print "path after VisibilityDijkstras:",path

		P_goal.reverse()

		for element in P_goal:
			if element not in temp_path:
				return None
			path.append(element)
		# print "path after P_goal:",path

		path.append((self.goal_config[0][0], self.goal_config[0][1]))

		# print "P_start:",P_start
		# print "P_goal:",P_goal
		# print "temp_path:",temp_path
		# print "path:",path

		return path


	def VisibilityDijkstras(self, N, E, start_config, goal_config):
		# print ("start DijkstraPlanner to goal")
		start_time = time.time()
		dijkstra_edges = self.GetDijsktraEdges(E)
		parent = {}

		A = {} # keeps track of expansions
		queue = [(0, start_config)]
		expansions = 0
		final_cost = 0
		debug = False

		while queue:
			cost, vertex1 = heappop(queue)
			if debug: print "popped vertex1:",vertex1,"cost:",cost

			if vertex1 == goal_config:
				final_cost = cost
				break

			if vertex1 not in A.keys():
				# print "not in A"
				A[vertex1] = cost
				vertex1_dict = dijkstra_edges[vertex1]
				if debug: print "vertex1_dict:",vertex1_dict
				for vertex2, temp_cost in vertex1_dict.items():
					if vertex2 not in A.keys():
						if debug: print "\tvertex2:",vertex2,"added cost:",(temp_cost + cost)
						heappush(queue, (cost + temp_cost, vertex2))
						if vertex2 in parent:
							if debug: print "\t\tin parent"
							other_parent_vertex2 = parent[vertex2]
							other_parent_dict = dijkstra_edges[other_parent_vertex2]
							previous_cost_vertex2 = other_parent_dict[vertex2]
							if previous_cost_vertex2 > (temp_cost + cost):
								if debug: print "\t\treplacing previous parent"
								parent[vertex2] = vertex1
							else:
								continue
						else:
							if debug: print "\t\tnew"
							parent[vertex2] = vertex1

			expansions+=1
			queue.sort()

		# print ("end DijkstraPlanner")
		# print("Seconds to complete DijkstraPlanner: " + str(time.time()- start_time))
		return self.ReconstructPath(parent, goal_config)


	def ReconstructPath(self, parent, current):
		total_path = [current]
		while current in parent:
			current = parent[current]
			total_path.append(current)

		total_path.reverse()
		plan = []
		for node in total_path:
			plan.append(node)

		return plan


	def GetDijsktraEdges(self, Edges):
		dijkstra_edges = {}
		self.AngleDict = {}
		
		for edge, neighbors in Edges.items():
			edge_x = edge[0]
			edge_y = edge[1]
			temp_edges = {}

			for neighbor in neighbors:
				neighbor_x = neighbor[0]
				neighbor_y = neighbor[1]

				cost = numpy.sqrt(numpy.square(edge_x - neighbor_x) + numpy.square(edge_y - neighbor_y))
				temp_edges[neighbor] = cost
			dijkstra_edges[edge] = temp_edges

		# dict of format:
		# key = [(x, y), theta]
		# val = dict of format:
		#     key = [((x_neighbor1, y_neighbor1), theta_neighbor1)]
		#	  val =	cost from (x, y), theta to (x_neighbor1, y_neighbor1), theta_neighbor1
		return dijkstra_edges