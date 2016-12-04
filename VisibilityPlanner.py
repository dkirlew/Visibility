#!/usr/bin/env python

import argparse, numpy, openravepy, time, math, random, ast

from collections import defaultdict
from heapq import heappop, heappush
from bintree import bintree
import heap

class VisibilityPlanner(object):

	def __init__(self, planning_env, visualize, width, height, robot_radius):
		self.planning_env = planning_env
		self.visualize = visualize
		self.width = width
		self.height = height
		self.robot_radius = robot_radius


	def Plan(env_config, start_config, goal_config):
		self.env_config = env_config
		self.start_config = start_config
		self.goal_config = goal_config

		Vertices = self.GetVerticesDict(env_config)
		Edges = {}
		for vertex in Vertices.keys(): 
			W = self.VisibleVertices(vertex, Vertices, start_config, goal_config)
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


	def GetVerticesDict(self, env_config):
		Vertices = {}

		for shape in env_config:
			if shape[0] == "R":
				#rectangle
				self.GetRectangleVertices(shape, Vertices)

			elif shape[0] == "L":
				#line
				self.GetLineVertices(shape, Vertices)

			elif shape[0] == "C":
				#circle
				self.GetCircleVertices(shape, Vertices)

			elif shape[0] == "A":
				#arc
				# self.GetArcVertices(shape, Vertices)
			else:
				print "Shape not recognized. (Should be R or L or C or A)"
				exit(0)

		Vertices[self.start_config] = []
		Vertices[self.goal_config] = []

		# dict is of form:
		#	key = 2D vertex
		#	val = list of adjacent 2D vertices
		return Vertices


	def VisibleVertices(self, vertex, Vertices):
		visible_vertices = []
		T = btree()

		thetas = self.FindSortedRelativeAngles(vertex, Vertices) # include start_config, goal_config
		i = 1
		prev_w = None

		while thetas:
			theta, w = heappop(thetas)

			if self.Visible(vertex, w, prev_w, i, T, W):
				self.AddRemoveEdges(w, Vertices, T)
				visible_vertices.append(w)

			prev_w = w
			i+=1

		return visible_vertices


	def FindSortedRelativeAngles(self, vertex, Vertices): # include start_config, goal_config

		relative_angles = []
		edge_x = edge[0]
		edge_y = edge[1]

		for neighbor in neighbors:
			neighbor_x = neighbor[0]
			neighbor_y = neighbor[1]
			relative_angles.heappush(self.FindRelativeAngle(edge_x, edge_y, neighbor_x, neighbor_y), neighbor)

		return relative_angles


	def Visible(self, vertex, neighbor, previous_neighbor, i, T, W):
		if self.CheckPathCollision(vertex, neighbor):
			return False
		if i == 1 or (previous_neighbor is not None and not self.PointOnEdge(vertex, neighbor, previous_neighbor)):
			closest_edge = T.min()
			if self.EdgesIntersect(vertex, neighbor, closest_edge):
				return False
			else:
				return True
		else:
			if previous_neighbor is not None and previous_neighbor not in W:
				return False
			else:
				for dist, edge in T.items():
					if self.EdgesIntersect(vertex, neighbor, edge):
						return False
				return True


	def CheckPathCollision(self, vertex, neighbor):
		vertex_x = vertex[0]
		vertex_y = vertex[1]
		neighbor_x = neighbor[0]
		neighbor_y = neighbor[1]

		dist = numpy.sqrt(float(numpy.square(vertex_x - neighbor_x) + numpy.square(vertex_y - neighbor_y)))

		for check_dist in range(0, dist + self.robot_radius / 2, self.robot_radius / 2):
			temp_loc = self.GetPointAtDistOnLine(vertex, neighbor, check_dist)
			if self.planning_env.CheckCollision(self.env_config, temp_loc[0], temp_loc[1]):
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


	def PointOnEdge(self, start_loc, end_loc, point):
		start_x = float(start_loc[0])
		start_y = float(start_loc[1])
		end_x = float(end_loc[0])
		end_y = float(end_loc[1])
		point_x = float(point_loc[0])
		point_y = float(point_loc[1])

		slope = (end_y - start_y) / (end_x - start_x)

		# point lies on line, so check if it lies within start and end points
		if (point_y - start_y) == (slope * (point_x - start_x)):
			if (start_x >= point_x and point_x >= end_x) or (end_x >= point_x and point_x >= start_x):
				if (start_y >= point_y and point_y >= end_y) or (end_y >= point_y and point_y >= start_y):
					return True

		return False


	def EdgesIntersect(self, edge1_start, edge1_end, edge2):
		edge2_start = edge[0]
		edge2_end = edge[1]

		x1 = float(edge1_start[0])
		y1 = float(edge1_start[1])
		x2 = float(edge1_end[0])
		y2 = float(edge1_end[1])
		x3 = float(edge2_start[0])
		y3 = float(edge2_start[1])
		x4 = float(edge2_end[0])
		y4 = float(edge2_end[1])

		# https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
		determinant = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

		if determinant != 0: # lines are not parallel nor coincident
			xp = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / determinant
			yp = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / determinant

			if ((x1 >= xp and xp >= x2) or (x1 <= xp and xp <= x2)) and ((x3 >= xp and xp >= x4) or (x3 <= xp and xp <= x4)):
				if ((y1 >= yp and yp >= y2) or (y1 <= yp and yp <= y2)) and ((y3 >= yp and yp >= y4) or (y3 <= yp and yp <= y4)):
					return True

		return False


	def AddRemoveEdges(self, vertex, Vertices, T):


	def GetRectangleVertices(self, shape, RectangleVertices):
		#http://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point

		x1 = float(shape[1][0]) # top left x
        y1 = float(shape[1][1]) # top left y
        x2 = float(shape[2][0]) # top right x
        y2 = float(shape[2][1]) # top right y
        x3 = float(shape[3][0]) # bottom right x
        y3 = float(shape[3][1]) # bottom right y
        x4 = float(shape[4][0]) # bottom left x
        y4 = float(shape[4][1]) # bottom left y

		#distance between physical vertex and buffered vertex
		dist = math.sqrt(2)*self.robot_radius

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

		RectangleVertices[(new_x1, new_y1)] = [(new_x2, new_y2), (new_x4, new_y4)]
		RectangleVertices[(new_x2, new_y2)] = [(new_x1, new_y1), (new_x3, new_y3)]
		RectangleVertices[(new_x3, new_y3)] = [(new_x2, new_y2), (new_x4, new_y4)]
		RectangleVertices[(new_x4, new_y4)] = [(new_x1, new_y1), (new_x3, new_y3)]


    def GetLineVertices(self, shape, LineVertices):

    	xs = float(shape[1][0]) # top left x
        ys = float(shape[1][1]) # top left y
        xe = float(shape[2][0]) # top right x
        ye = float(shape[2][1]) # top right 

    	theta = self.FindRelativeAngle(xs, ys, xe, ye)

    	if theta > math.pi/2:
    		theta_rotated = math.pi - theta
    	elif theta < math.pi/2:
    		theta_rotated = -theta

    	xm = abs(xe-xs)/2 + min(xs, xe)
    	ym = abs(ye-ys)/2 + min(ys, ye)

    	xs_rotated = math.cos(theta_rotated) * (xs - xm) - math.sin(theta_rotated) * (ys - ym) + xm
	    ys_rotated = math.sin(theta_rotated) * (xs - xm) + math.cos(theta_rotated) * (ys - ym) + ym
	    xe_rotated = math.cos(theta_rotated) * (xe - xm) - math.sin(theta_rotated) * (ye - ym) + xm
	    ye_rotated = math.sin(theta_rotated) * (xe - xm) + math.cos(theta_rotated) * (ye - ym) + ym

	    x1 = xs_rotated - self.robot_radius
	    y1 = ys_rotated - self.robot_radius
	    x2 = xs_rotated + self.robot_radius
	    y2 = ys_rotated - self.robot_radius
	    x3 = xs_rotated + self.robot_radius
	    y3 = ys_rotated + self.robot_radius
	    x4 = xs_rotated - self.robot_radius
	    y4 = ys_rotated + self.robot_radius

	    x1_rotated = math.cos(theta) * (x1 - xm) - math.sin(theta) * (y1 - ym) + xm
	    y1_rotated = math.sin(theta) * (x1 - xm) + math.cos(theta) * (y1 - ym) + ym
	    x2_rotated = math.cos(theta) * (x2 - xm) - math.sin(theta) * (y2 - ym) + xm
	    y2_rotated = math.sin(theta) * (x2 - xm) + math.cos(theta) * (y2 - ym) + ym
	    x3_rotated = math.cos(theta) * (x3 - xm) - math.sin(theta) * (y3 - ym) + xm
	    y3_rotated = math.sin(theta) * (x3 - xm) + math.cos(theta) * (y3 - ym) + ym
	    x4_rotated = math.cos(theta) * (x4 - xm) - math.sin(theta) * (y4 - ym) + xm
	    y4_rotated = math.sin(theta) * (x4 - xm) + math.cos(theta) * (y4 - ym) + ym

		LineVertices[(x1_rotated, y1_rotated)] = [(x2_rotated, y2_rotated), (x4_rotated, y4_rotated)]
		LineVertices[(x2_rotated, y2_rotated)] = [(x1_rotated, y1_rotated), (x3_rotated, y3_rotated)]
		LineVertices[(x3_rotated, y3_rotated)] = [(x2_rotated, y2_rotated), (x4_rotated, y4_rotated)]
		LineVertices[(x4_rotated, y4_rotated)] = [(x1_rotated, y1_rotated), (x3_rotated, y3_rotated)]


	def GetCircleVertices(self, shape, CircleVertices):

		xc = float(shape[1][0]) 
        yc = float(shape[1][1])
        circle_radius = float(shape[2])

       	buffer_dist = circle_radius + self.robot_radius

        x_start = xc + buffer_dist/math.cos(math.pi/8)
        y_start = yc

        x_curr = x_start
        y_curr = y_start

        CircleVertices[x_curr, y_curr] = []

        for i in range(1, 8):
        	theta = math.pi/4 * i
	        x_temp = math.cos(theta) * (x_curr - xc) - math.sin(theta) * (y_curr - yc) + xc
    		y_temp = math.sin(theta) * (x_curr - xc) + math.cos(theta) * (y_curr - yc) + yc
    		CircleVertices[(x_temp, y_temp)] = [(x_curr, y_curr)]
    		CircleVertices[(x_curr, y_curr)].append((x_temp, y_temp))
			x_curr = x_temp
			y_curr = y_temp


		CircleVertices[(x_curr, y_curr)].append((x_start, y_start))
		CircleVertices[(x_start, y_start)].append((x_curr, y_curr))


	def GetArcVertices(self, shape, ArcVertices):

		arc_radius = float(shape[2])
		xc = float(shape[1][0]) + arc_radius
        yc = float(shape[1][1]) + arc_radius
        start_angle = float(shape[3])
        stop_angle = float(shape[4])
        

       	buffer_dist = arc_radius + self.robot_radius

        x_start = xc + buffer_dist/math.cos(math.pi/8)
        y_start = yc

        if start_angle > stop_angle:
        	ArcVertices[(x_curr, y_curr)] = []

        x_curr = x_start
        y_curr = y_start

        # add vertices only if they're not in gap of arc
        for i in range(1, 8):
        	theta = math.pi/4 * i
	        x_temp = math.cos(theta) * (x_curr - xc) - math.sin(theta) * (y_curr - yc) + xc
    		y_temp = math.sin(theta) * (x_curr - xc) + math.cos(theta) * (y_curr - yc) + yc

    		if start_angle > stop_angle:
    			if theta > start_angle or theta < stop_angle:
    				if (x_curr, y_curr) in ArcVertices.keys():
    					ArcVertices[(x_temp, y_temp)] = [(x_curr, y_curr)]
    					ArcVertices[(x_curr, y_curr)].append((x_temp, y_temp))
    				else:
    					ArcVertices[(x_temp, y_temp)] = []
    		else:
    			if theta > start_angle and theta < stop_angle:
    				if (x_curr, y_curr) in ArcVertices.keys():
    					ArcVertices[(x_temp, y_temp)] = [(x_curr, y_curr)]
    					ArcVertices[(x_curr, y_curr)].append((x_temp, y_temp))
    				else:
    					ArcVertices[(x_temp, y_temp)] = []

			x_curr = x_temp
			y_curr = y_temp

		if(x_curr, y_curr) in ArcVertices.keys() and (x_start, y_start) in ArcVertices.keys():
			ArcVertices[(x_start, y_start)] = [(x_curr, y_curr)]
			ArcVertices[(x_curr, y_curr)].append((x_start, y_start))



		if start_angle > stop_angle:
			mid_angle = stop_angle + (start_angle - stop_angle) / 2
		else:
			mid_angle = ((stop_angle - start_angle) / 2 + math.pi) % (2 * math.pi)


        x_mid = math.cos(mid_angle) * (x_start - xc) - math.sin(mid_angle) * (y_start - yc) + xc
		y_mid = math.sin(mid_angle) * (x_start - xc) + math.cos(mid_angle) * (y_start - yc) + yc

		ArcVertices.append([x_mid, y_mid])
		ArcVertices.append([xc, yc])

	       
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