#!/usr/bin/env python

import argparse, numpy, openravepy, time, math, random, ast

class VisibilityPlanner(object):

	def __init__(self, planning_env, visualize, width, height, robot_radius):
		self.planning_env = planning_env
		self.visualize = visualize
		self.width = width
		self.height = height
		self.robot_radius = robot_radius


	def Plan(env_config, start_config, goal_config):
		Vertices = self.GetVertices(env_config, start_config, goal_config)
		Edges = None
		for vertex in Vertices: 
			W = self.VisibleVertices(vertex, env_config, start_config, goal_config)
			for w in W:
				#Add [w, v]?
				Edges.append([vertex, w])

		return Vertices, Edges

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

		RectangleVertices = []

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

       







