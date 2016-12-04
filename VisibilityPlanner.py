#!/usr/bin/env python

import argparse, numpy, openravepy, time, math, random, ast

from collections import defaultdict
from heapq import heappop, heappush
import heap

class VisibilityPlanner(object):

	def __init__(self, planning_env, visualize, width, height, robot_radius):
		self.planning_env = planning_env
		self.visualize = visualize
		self.width = width
		self.height = height
		self.robot_radius = robot_radius


	def Plan(env_config, start_config, goal_config):
		Vertices = self.GetVertices(env_config, start_config, goal_config)
		Edges = {}
		for vertex in Vertices: 
			W = self.VisibleVertices(vertex, env_config, start_config, goal_config)
			for w in W:
				if vertex in Edges:
					Edges[vertex].append(w)
				else:
					Edges[vertex] = [w]

				if w in Edges:
					Edges[w].append(vertex)
				else:
					Edges[w] = [vertex]

		plan = self.VisibilityDijkstras(Vertices, Edges, start_config, goal_config)

		return plan


	def GetVertices(self, env_config, start_config, goal_config):
		Vertices = []

		for shape in env_config:
			if shape[0] == "R":
				#rectangle
				Vertices.append(self.GetRectangleVertices(shape))

			elif shape[0] == "L":
				#line
			elif shape[0] == "C":
				#circle
			elif shape[0] == "A":
				#arc
			else:
				print "Shape not recognized. (Should be R or L or C or A)"
				exit(0)
		return Vertices


	def GetRectangleVertices(self, shape):
		#http://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point

		RectangleVertices = []

		x1 = float(shape[1][0]) # top left x
        y1 = float(shape[1][1]) # top left y
        x2 = float(shape[2][0]) # top right x
        y2 = float(shape[2][1]) # top right y
        x3 = float(shape[3][0]) # bottom right x
        y3 = float(shape[3][1]) # bottom right y
        x4 = float(shape[4][0]) # bottom left x
        y4 = float(shape[4][1]) # bottom left y

		#distance between physical vertex and buffered vertex
		dist = math.sqrt(2)*50

		vector13 = [x1-x3, y1-y3]
		vectorMagnitude13 = math.sqrt(numpy.square(x1-x3) + numpy.square(y1-y3))
		unitVector13 = [vector13[0]/vectorMagnitude13, vector13[1]/vectorMagnitude13]

		[new_x1, new_y1] = [x1 + dist*unitVector13[0], y1 + dist*unitVector13[1]]
		[new_x3, new_y3] = [x3 - dist*unitVector13[0], y3 - dist*unitVector13[1]]

		vector24 = [x2-x4, y2-y4]
		vectorMagnitude24 = math.sqrt(numpy.square(x2-x4) + numpy.square(y2-y4))
		unitVector24 = [vector24[0]/vectorMagnitude24, vector24[1]/vectorMagnitude24]

		[new_x2, new_y2] = [x2 + dist*unitVector24[0], y2 + dist*unitVector24[1]] 
		[new_x4, new_y4] = [x4 - dist*unitVector24[0], y4 - dist*unitVector24[1]] 

		RectangleVertices.append([new_x1, new_y1])
		RectangleVertices.append([new_x2, new_y2])
		RectangleVertices.append([new_x3, new_y3])
		RectangleVertices.append([new_x4, new_y4])

        return RectangleVertices

	       
	def VisibilityDijkstras(self, Vertices, Edges, start_config, goal_config):
	        print "start DijkstraPlanner to goal"
		start_time = time.time()
		dijkstra_edges = self.GetDijsktraEdges(Edges)
		parent = {}

		A = [None] * len(dijkstra_edges)
		queue = [(0, start_config)]
		expansions = 0

		while queue:
			cost, vertex1 = heappop(queue)

			if vertex1 == goal_config:
				final_cost = cost
				break

			if A[vertex1] is None:
				A[vertex1] = cost
				for vertex2, temp_cost in dijkstra_edges[vertex1].items():
					if A[vertex2] is None:
						heappush(queue, (cost + temp_cost, vertex2))
						parent[vertex2] = vertex1
			expansions+=1


		print "end DijkstraPlanner"
	    print("Seconds to complete DijkstraPlanner: " + str(time.time()- start_time))
	    return final_cost, self.ReconstructPath(parent, goal_id), expansions


	def GetDijsktraEdges(self, Edges):
		dijkstra_edges = {}
		Edges3D = self.Get3DEdges(Edges) # dict of format [(x, y), theta] = [((x_neighbor1, y_neighbor1), theta_neighbor1]

		for edge, neighbors in Edges3D.items():
			temp_edges = {}
			for neighbor in neighbors:
				temp_edges[neighor] = self.CostOfMove(edge, neighbor)
			dijkstra_edges[edge] = temp_edges

		# dict of format:
		# key = [(x, y), theta]
		# val = dict of format:
		#     key = [((x_neighbor1, y_neighbor1), theta_neighbor1)]
		#	  val =	cost from (x, y), theta to (x_neighbor1, y_neighbor1), theta_neighbor1
		return dijkstra_edges


	def Get3DEdges(self, Edges):
		Edges3D = {}

		for edge, neighbors in Edges.keys():
			neighbor_thetas = self.FindRelativeAngles(edge, neighbors)

			for neighbor in neighbors:
				theta_edge_to_neighor = neighbor_thetas[neighbor]

				# if facing neighboring vertex, neighbor is that vertex
				Edges3D[(edge, theta_edge_to_neighor)] = [(neighbor, theta_edge_to_neighor)]
				
				for neighbor_theta in neighbor_thetas:
					# if facing neighboring vertex, neighbor can also be rotation towards other vertices
					if theta_edge_to_neighor != neighbor_theta:
						Edges3D[(edge, theta_edge_to_neighor)].append((edge, neighor_theta))

					# if just arrived from vertex (have opp angle), neighbors are turning to visible vertices, including turning around pi radians
					if (edge, (theta_edge_to_neighor + math.pi)%(2 * math.pi)) in Edges:
						Edges3D[(edge, (theta_edge_to_neighor + math.pi)%(2 * math.pi))].append((neighbor, theta_edge_to_neighor))
					else:
						Edges3D[(edge, (theta_edge_to_neighor + math.pi)%(2 * math.pi))] = [(neighbor, theta_edge_to_neighor)]


		# dict of format:
		# key = [(x, y), theta]
		# val = [((x_neighbor1, y_neighbor1), theta_neighbor1), ((x_neighbor2, ....]
		return Edges3D


	def FindRelativeAngles(self, edge, neighbors):
		relative_angles = {}
		edge_x = edge[0]
		edge_y = edge[1]

		for neighbor in neighbors:
			neighbor_x = neighbor[0]
			neighbor_y = neighbor[1]
			relative_angles[neighbor] = self.FindRelativeAngle(edge_x, edge_y, neighbor_x, neighbor_y)

		return relative_angles


	# calculate angle from origin to point p
	def FindRelativeAngle(self, o_x, o_y, p_x, p_y):

		# y axis is flipped
		origin_x = float(o_x)
		origin_y = float(o_y * -1)
		point_x = float(p_x)
		point_y = float(p_y * -1)

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

        return relative_theta


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
   			cost/= numpy.sqrt(float(numpy.square(self.height) + numpy.square(self.width)))
   		# rotational distance, scaled by longest possible rotation - pi
   		else:
   			cost = abs(edge_theta - neighbor_theta)
   			cost/= math.pi
   			
   		return cost


	def ReconstructPath(self, parent, current):
	    total_path = [current]
	    while current in parent:
	        current = parent[current]
	        total_path.append(current)

	    total_path.reverse()
	    plan = []
	    for node in total_path:
	        # plan.append(self.NodeIdToGridCoord(node))
	        plan.append(node)

	    return plan