import argparse, numpy, openravepy, time, math, random
from heapq import heappop, heappush

class RRTTree(object):
	
	def __init__(self, start_coord):
		self.Nodes2D = [start_coord]
		self.NodeParent = {}


	def GetNearestNode(self, random_coord):
		Distances = []
		for node in self.Nodes2D:
			dist = round(self.ComputeDistance(random_coord, node),2)
			heappush(Distances, (dist, node))

		nearest = heappop(Distances)
		nearest_node = nearest[1]
		
		return nearest_node
			

	def AddEdge(self, nearest_node, new_coord):
		self.Nodes2D.append(new_coord)
		self.NodeParent[new_coord] = [nearest_node]


	def ComputeDistance(self, random_config, node):
		dist = numpy.sqrt(numpy.square(random_config[0]-node[0])+numpy.square(random_config[1]-node[1]))
		return dist