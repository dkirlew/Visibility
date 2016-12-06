#!/usr/bin/env python

import argparse, numpy, openravepy, time, math, random, ast

from collections import defaultdict
from heapq import heappop, heappush
# import heap

class VisibilityPlanner(object):

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

		Vertices = self.GetVerticesDict(env_config)
		Edges = {}
		for vertex in Vertices.keys():
			W = self.VisibleVertices(vertex, Vertices)
			for w in W:
				if vertex in Edges:
					# print "edge vertex:", vertex
					if w not in Edges[vertex]:
						Edges[vertex].append(w)
				else:
					# print "edge vertex:", vertex
					Edges[vertex] = [w]

				if w in Edges:
					# print "edge w:",w
					if vertex not in Edges[w]:
						Edges[w].append(vertex)
				else:
					# print "edge w:",w
					Edges[w] = [vertex]

		len_path, plan, construct_time = self.VisibilityDijkstras(Vertices, Edges)
		num_nodes = len(Vertices.keys()) - 2 # start and goal don't count

		return Vertices, Edges, plan, construct_time, num_nodes, len_path, 0


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
				continue
			else:
				print ("Shape not recognized. (Should be R or L or C or A)")
				exit(0)

		start_x = self.start_config[0][0]
		start_y = self.start_config[0][1]
		goal_x = self.goal_config[0][0]
		goal_y = self.goal_config[0][1]

		Vertices[(start_x, start_y)] = []
		Vertices[(goal_x, goal_y)] = []

		# dict is of form:
		#	key = 2D vertex
		#	val = list of adjacent 2D vertices
		return Vertices


	def VisibleVertices(self, vertex, Vertices):
		W = []
		T = {}
		start_x = self.start_config[0][0]
		start_y = self.start_config[0][1]
		goal_x = self.goal_config[0][0]
		goal_y = self.goal_config[0][1]

		thetas = self.FindSortedRelativeAngles(vertex, Vertices) # include start_config, goal_config
		i = 1
		prev_w = None

		while thetas:
			(theta, dist), w = heappop(thetas)

			if vertex != w:
				if self.Visible(vertex, w, prev_w, i, T, W):
					self.AddRemoveEdges(vertex, w, Vertices[w], T)
					W.append(w)

				prev_w = w
				i+=1

		return W


	def FindSortedRelativeAngles(self, vertex, Vertices): 
		# print "vertex:", vertex
		relative_angles = []
		vertex_x = vertex[0]
		vertex_y = vertex[1]
		start_x = self.start_config[0][0]
		start_y = self.start_config[0][1]
		goal_x = self.goal_config[0][0]
		goal_y = self.goal_config[0][1]

		# Vertices includes start_config and goal_config
		for neighbor, val in Vertices.items():
			if neighbor != vertex:
				# print "neighbor:",neighbor
				neighbor_x = neighbor[0]
				neighbor_y = neighbor[1]

				# print "vertex_x:",vertex_x
				# print "vertex_y:",vertex_y
				# print "neighbor_x:",neighbor_x
				# print "neighbor_y:",neighbor_y
				relative_angle = self.FindRelativeAngle(vertex_x, vertex_y, neighbor_x, neighbor_y)
				dist = numpy.sqrt(float(numpy.square(vertex_x - neighbor_x) + numpy.square(vertex_y - neighbor_y)))

				heappush(relative_angles, ((relative_angle, dist), neighbor))

		return relative_angles


	def Visible(self, vertex, neighbor, previous_neighbor, i, T, W):
		if self.CheckPathCollision(vertex, neighbor):
			return False
		if i == 1:
			return True
		if previous_neighbor is not None and not self.PointOnEdge(vertex, neighbor, previous_neighbor):
			if len(T.keys()) == 0:
				return True
			else:
				closest_edge = T[min(T.keys())]
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
			if self.planning_env.CheckCollision(self.env_config, temp_loc[0], temp_loc[1]):
				return True
			check_dist+=(self.robot_radius / 2.0)
		
		if self.planning_env.CheckCollision(self.env_config, neighbor_x, neighbor_y):
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


	def PointOnEdge(self, start_loc, end_loc, point_loc):
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
		edge2_start = edge2[0]
		edge2_end = edge2[1]

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

			if (max(x1, x2) > xp and xp > min(x1, x2)) and (max(x3, x4) > xp and xp > min(x3, x4)):
				if (max(y1, y2) > yp and yp >=min(y1, y2)) and (max(y3, y4) > yp and yp > min(y3, y4)):
					return True

		return False


	def AddRemoveEdges(self, start_point, vertex, neighbors, T):
		for neighbor in neighbors:
			temp_key = self.planning_env.DistPointToLine(start_point[0], start_point[1], vertex[0], vertex[1], neighbor[0], neighbor[1])
			
			if self.Clockwise(start_point, vertex, neighbor):
				if temp_key not in T:
					T[temp_key] = [vertex, neighbor]
			else:
				if temp_key in T:
					del T[temp_key]
		return


	def Clockwise(self, start_point, end_point, vertex):
		start_x = float(start_point[0])
		start_y = float(start_point[1])
		end_x = float(end_point[0])
		end_y = float(end_point[1])
		vertex_x = float(vertex[0])
		vertex_y = float(vertex[1])

		start_end_theta = self.FindRelativeAngle(start_x, start_y, end_x, end_y)
		start_vertex_theta = self.FindRelativeAngle(start_x, start_y, vertex_x, vertex_y)

		if start_vertex_theta > start_end_theta:
			return True

		else:
			start_y*=-1
			end_y*=-1
			vertex_y*=-1
			if end_x != vertex_x:
				slope = (end_y - vertex_y) / (end_x - vertex_x)
			else: # check if vertex is above or below end point
				if start_end_theta >= (math.pi / 2) and start_end_theta <= (3* math.pi / 2):
					if end_y > vertex_y: # e above v
						return False
					else:
						return True # should never happen because start_vertex_theta < start_end_theta
				else:
					if end_y < vertex_y: # e below v
						return False
					else:
						return True # should never happen because above

			if start_end_theta > math.pi:
				# check if slope is positive and start point is below line from end to vertex
				if slope > 0:
					A = -(vertex_y - end_y)
					B = vertex_x - end_x
					C = -(A * end_x + B * end_y)
					D = A * start_x + B * start_y + C

					if D > 0: # lies to the left
						return True
				else:
					A = -(end_y - vertex_y)
					B = end_x - vertex_x
					C = -(A * vertex_x + B * vertex_y)
					D = A * start_x + B * start_y + C

					if D > 0:
						return True
		return False


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



		RectangleVertices[(new_x1, new_y1)] = [(new_x2, new_y2), (new_x4, new_y4)]
		RectangleVertices[(new_x2, new_y2)] = [(new_x1, new_y1), (new_x3, new_y3)]
		RectangleVertices[(new_x3, new_y3)] = [(new_x2, new_y2), (new_x4, new_y4)]
		RectangleVertices[(new_x4, new_y4)] = [(new_x1, new_y1), (new_x3, new_y3)]

		return


	def GetLineVertices(self, shape, LineVertices):
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


		LineVertices[(x1_rotated, y1_rotated)] = [(x2_rotated, y2_rotated), (x4_rotated, y4_rotated)]
		LineVertices[(x2_rotated, y2_rotated)] = [(x1_rotated, y1_rotated), (x3_rotated, y3_rotated)]
		LineVertices[(x3_rotated, y3_rotated)] = [(x2_rotated, y2_rotated), (x4_rotated, y4_rotated)]
		LineVertices[(x4_rotated, y4_rotated)] = [(x1_rotated, y1_rotated), (x3_rotated, y3_rotated)]

		return


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
			# print "circle loop (x_temp, y_temp):",(x_temp, y_temp)
			# print "circle loop (x_curr, y_curr):",(x_curr, y_curr)

			CircleVertices[(x_temp, y_temp)] = [(x_curr, y_curr)]
			CircleVertices[(x_curr, y_curr)].append((x_temp, y_temp))
			x_curr = x_temp
			y_curr = y_temp

		# print "circle (x_curr, y_curr):",(x_curr, y_curr)
		# print "circle (x_start, y_start):",(x_start, y_start)

		CircleVertices[(x_curr, y_curr)].append((x_start, y_start))
		CircleVertices[(x_start, y_start)].append((x_curr, y_curr))

		return


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

		return

		   
	def VisibilityDijkstras(self, Vertices, Edges):
		# print ("start DijkstraPlanner to goal")
		start_time = time.time()
		dijkstra_edges = self.GetDijsktraEdges(Edges)
		parent = {}

		A = {} # keeps track of expansions
		queue = [(0, self.start_config)]
		expansions = 0
		final_cost = 0

		while queue:
			cost, vertex1 = heappop(queue)
			# print "popped vertex1:",vertex1,"cost:",cost

			if vertex1 == self.goal_config:
				final_cost = cost
				break

			if vertex1 not in A.keys():
				# print "not in A"
				A[vertex1] = cost
				vertex1_dict = dijkstra_edges[vertex1]
				# print "vertex1_dict:",vertex1_dict
				for vertex2, temp_cost in vertex1_dict.items():
					if vertex2 not in A.keys():
						# print "\tvertex2:",vertex2,"added cost:",(temp_cost + cost)
						heappush(queue, (cost + temp_cost, vertex2))
						if vertex2 in parent:
							# print "\t\tin parent"
							other_parent_vertex2 = parent[vertex2]
							other_parent_dict = dijkstra_edges[other_parent_vertex2]
							previous_cost_vertex2 = other_parent_dict[vertex2]
							if previous_cost_vertex2 > (temp_cost + cost):
								# print "\t\treplacing previous parent"
								parent[vertex2] = vertex1
							else:
								continue
						else:
							# print "\t\tnew"
							parent[vertex2] = vertex1
					# elif temp_cost < A[vertex2]:

			expansions+=1
			queue.sort()

		construct_time = time.time() - start_time
		return final_cost, self.ReconstructPath(parent, self.goal_config), construct_time


	def GetDijsktraEdges(self, Edges):
		dijkstra_edges = {}
		self.AngleDict = {}
		Edges3D = self.Get3DEdges(Edges) # dict of format [(x, y), theta] = [((x_neighbor1, y_neighbor1), theta_neighbor1]

		test = ((434.55, 672.28), 4.15)

		for edge, neighbors in Edges3D.items():
			temp_edges = {}

			for neighbor in neighbors:
				temp_edges[neighbor] = self.planning_env.CostOfMove(edge, neighbor)
			# print "edge in dict dict:", edge
			dijkstra_edges[edge] = temp_edges

		# dict of format:
		# key = [(x, y), theta]
		# val = dict of format:
		#     key = [((x_neighbor1, y_neighbor1), theta_neighbor1)]
		#	  val =	cost from (x, y), theta to (x_neighbor1, y_neighbor1), theta_neighbor1
		return dijkstra_edges


	def Get3DEdges(self, Edges):
		Edges3D = {}

		start_x = float(self.start_config[0][0])
		start_y = float(self.start_config[0][1])
		start_theta = self.start_config[1]
		goal_x = float(self.goal_config[0][0])
		goal_y = float(self.goal_config[0][1])
		goal_theta = self.goal_config[1]

		start2D = (start_x, start_y)
		goal2D = (goal_x, goal_y)
		# print "start2D:",start2D

		# print "goal2D:",goal2D

		for edge, neighbors in Edges.items():
			# print "edge or vertex v is:",edge
			neighbor_thetas = self.FindRelativeAngles(edge, neighbors)

			if edge == (512.01, 936.7):
				print "now debugging"
				print "neighbors of p1:",neighbors
			# 	debug = True
			# else:
			# 	debug = False

			for neighbor in neighbors:
				# if debug:
				# 	print "now looking at neighbor",neighbor
				theta_edge_to_neighbor = neighbor_thetas[neighbor]
				# print "theta_edge_to_neighbor:",theta_edge_to_neighbor

				if edge != goal2D and edge != start2D:
					if neighbor != start2D: # do not 1. face neighbor start, 2. go to neighbor start
						# if facing neighboring vertex, neighbor is that vertex
						if (edge, theta_edge_to_neighbor) not in Edges:
							Edges3D[(edge, theta_edge_to_neighbor)] = [(neighbor, theta_edge_to_neighbor)]
						else:
							Edges3D[(edge, theta_edge_to_neighbor)].append((neighbor, theta_edge_to_neighbor))
					# 	if debug:
					# 		print "move to neighbor, theta_edge_to_neighbor:", theta_edge_to_neighbor
					# 		print "\t\tEdges3D[",(edge, theta_edge_to_neighbor),"]:",Edges3D[(edge, theta_edge_to_neighbor)]
					# elif debug:
					# 	print "neighbor",neighbor,"is start"

				
					for possible_neighbor, neighbor_theta in neighbor_thetas.items():
						# if debug:
							# print "\tnow looking at possible_neighbor",possible_neighbor
						# if foo == start2D:
						# 	print "found start in neighbors"
						# 	print "theta from foo to start:", neighbor_theta
						# 	print "theta from start to foo:", round((neighbor_theta + math.pi)%(2 * math.pi), 2)

						# print "neighbor_theta:",neighbor_theta
						# don't face start and rotate elsewhere, don't face elsewhere and rotate start
						if neighbor != start2D and possible_neighbor != start2D:
							# if facing neighboring vertex, neighbor can also be rotation towards other vertices
							if theta_edge_to_neighbor != neighbor_theta:
								if (edge, theta_edge_to_neighbor) not in Edges3D:
									Edges3D[(edge, theta_edge_to_neighbor)] = [(edge, neighbor_theta)]
								else:
									if (edge, neighbor_theta) not in Edges3D[(edge, theta_edge_to_neighbor)]:
										Edges3D[(edge, theta_edge_to_neighbor)].append((edge, neighbor_theta))
								# if debug:
								# 	print "\t\tadded to self, theta_edge_to_neighbor(",theta_edge_to_neighbor,") to possible neighbor(",possible_neighbor,") neighbor_theta:",neighbor_theta
								# 	print "\t\tEdges3D[",(edge, theta_edge_to_neighbor),"]:",Edges3D[(edge, theta_edge_to_neighbor)]
							# elif debug:
							# 	print "\t\tequal to self"
						# elif debug:
						# 	print "\t\teither neighbor",neighbor,"or possible_neighbor",possible_neighbor,"is start"


						# do not include start ID in possible neighbors to disallow backtracking to start only (all other edges are bidirectional (except the goal (but that's something else)))
						if neighbor != start2D:
							# if just arrived from vertex (have opp angle), neighbors are turning to visible vertices, including turning around pi radians
							if (edge, round((neighbor_theta + math.pi)%(2 * math.pi), 2)) in Edges3D:
								Edges3D[(edge, round((neighbor_theta + math.pi)%(2 * math.pi), 2))].append((edge, theta_edge_to_neighbor))
								# if debug: print "\t\t\tthis is a known instance of",(edge, round((neighbor_theta + math.pi)%(2 * math.pi), 2))
							else:
								Edges3D[(edge, round((neighbor_theta + math.pi)%(2 * math.pi), 2))] = [(edge, theta_edge_to_neighbor)]
								# if debug: print "\t\t\tthis is a new instance of",(edge, round((neighbor_theta + math.pi)%(2 * math.pi), 2))
							# if debug:
							# 	print "\t\tadded to incoming neighbor, theta_edge_to_neighbor.  possible_neighbor:",possible_neighbor
							# 	print "\t\tEdges3D[",(edge, round((neighbor_theta + math.pi)%(2 * math.pi), 2)),"]:",Edges3D[(edge, round((neighbor_theta + math.pi)%(2 * math.pi), 2))]
						# elif debug:
						# 	print "\t\tincoming neighbor",neighbor,"is start"

				elif edge == start2D:
					# print "start neighbor:", neighbor
					# print "neighbor theta:", theta_edge_to_neighbor
					if (edge, start_theta) not in Edges3D:
						Edges3D[(edge, start_theta)] = [(edge, theta_edge_to_neighbor)]
						# print "start added",(edge, theta_edge_to_neighbor)
					else:
						Edges3D[(edge, start_theta)].append((edge, theta_edge_to_neighbor))
						# print "start added",(edge, theta_edge_to_neighbor)

					if (edge, theta_edge_to_neighbor) not in Edges3D:
						Edges3D[(edge, theta_edge_to_neighbor)] = [(neighbor, theta_edge_to_neighbor)]
					elif (neighbor, theta_edge_to_neighbor) not in Edges3D[(edge, theta_edge_to_neighbor)]:
						Edges3D[(edge, theta_edge_to_neighbor)].append((neighbor, theta_edge_to_neighbor))


					for foo, neighbor_theta in neighbor_thetas.items():
						if theta_edge_to_neighbor != neighbor_theta:
							if (edge, theta_edge_to_neighbor) not in Edges3D:
								Edges3D[(edge, theta_edge_to_neighbor)] = [(edge, neighbor_theta)]
							elif (edge, neighbor_theta) not in Edges3D[(edge, theta_edge_to_neighbor)]:
								Edges3D[(edge, theta_edge_to_neighbor)].append((edge, neighbor_theta))


				else: # is Goal
					# from incoming neighbor, can only rotate to goal theta
					if (edge, round((theta_edge_to_neighbor + math.pi)%(2 * math.pi), 2)) not in Edges3D:
						Edges3D[(edge, round((theta_edge_to_neighbor + math.pi)%(2 * math.pi), 2))] = [(edge, goal_theta)]
					else:
						Edges3D[(edge, round((theta_edge_to_neighbor + math.pi)%(2 * math.pi), 2))].append((edge, goal_theta))


		# dict of format:
		# key = [(x, y), theta]
		# val = [((x_neighbor1, y_neighbor1), theta_neighbor1), ((x_neighbor2, ....]
		return Edges3D


	def FindRelativeAngles(self, edge, neighbors):
		# print "edge:",edge
		relative_angles = {}
		edge_x = edge[0]
		edge_y = edge[1]

		for neighbor in neighbors:
			# print "neighbor:",neighbor
			neighbor_x = neighbor[0]
			neighbor_y = neighbor[1]
			# print "edge_x:",edge_x
			# print "edge_y:",edge_y
			# print "neighbor_x:",neighbor_x
			# print "neighbor_y:",neighbor_y

			if ((edge_x, edge_y), (neighbor_x, neighbor_y)) in self.AngleDict:
				relative_angles[neighbor] = self.AngleDict[((edge_x, edge_y), (neighbor_x, neighbor_y))]
			else:
				temp_angle = self.FindRelativeAngle(edge_x, edge_y, neighbor_x, neighbor_y)
				self.AngleDict[((edge_x, edge_y), (neighbor_x, neighbor_y))] = temp_angle
				self.AngleDict[((neighbor_x, neighbor_y), (edge_x, edge_y))] = round((temp_angle + math.pi) % (2 * math.pi), 2)

				relative_angles[neighbor] = temp_angle

		return relative_angles


	# calculate angle from origin to point p
	def FindRelativeAngle(self, o_x, o_y, p_x, p_y, should_round = True):

		# y axis is flipped
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

		# print "relative_theta:",relative_theta
		if should_round:
			return round(relative_theta, 2)
		else:
			return relative_theta


	def ReconstructPath(self, parent, current):
		# print "final plan"
		total_path = [current]
		while current in parent:
			current = parent[current]
			total_path.append(current)

		total_path.reverse()
		plan = []
		for node in total_path:
			# plan.append(self.NodeIdToGridCoord(node))
			# print "node in plan:",node
			plan.append(node)

		return plan