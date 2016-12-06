import argparse, numpy, openravepy, time, math, random
from heapq import heappop, heappush

class RRTTree(object):
	
	def __init__(self, start_coord):
		
		# self.planning_env = planning_env
		self.Nodes = [start_coord]
		self.NodeParent = {}

		print "Initial Tree :", self.Nodes

	def GetRootId(self):
		return 0

	def GetNearestNode(self, random_coord):
		Distances = []
		# print "Nodes : ", self.Nodes
		print len(self.Nodes)
		for node in self.Nodes:
			dist = round(self.ComputeDistance(random_coord, node),2)
			heappush(Distances, (dist, node))

		# print "Distances Heap : ", Distances

		nearest = heappop(Distances)
		shortest_dist = nearest[0]
		nearest_node = nearest[1]
		# print "random_coord :", random_coord
		# print "Distances heap :", Distances
		# print "nearest_node :", nearest
		
		return shortest_dist, nearest_node
			

	def AddEdge(self, nearest_node, new_coord):
		self.Nodes.append(new_coord)
		self.NodeParent[new_coord] = nearest_node



	def ComputeDistance(self, random_config, node):
		dist = numpy.sqrt(numpy.square(random_config[0]-node[0])+numpy.square(random_config[1]-node[1]))
		return dist