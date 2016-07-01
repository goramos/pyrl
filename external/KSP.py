'''
KSP v1.21

Created on February 10, 2014 by Gabriel de Oliveira Ramos <goramos@inf.ufrgs.br>

Versions history:
v1.0 (10-Feb-2014) -  Creation.
v1.1 (20-Apr-2014) -  Adjusted error related with the creation of root and spur paths.
					  By definition, the spur node must be both in root AND spur paths
					  (previously the spur node was not in root path, and some problems
					  were identified in Ortuzar network for AM pair and K=4).
v1.2 (12-May-2014) -  Call of algorithm was changed from 'main(...)' to 'run(...)', and
					  parameters 'origin' and 'destination' were replaced by 'OD_list'
					  (a list of OD-pairs). Furthermore, the list of parameters of the 
					  command line call (__main__) was changed accordingly.
					  Finally, the output text was changed according to the specification
					  defined by Dr. Ana Bazzan.
v1.21 (5-Jun-2014) -  Created the function getKRoutes(graph_file, origin, destination, K)
                      to be called externally by another applications. The function returns,
                      for a given origin and destination, the K shortest routes in a list
                      format. The list structure was specified in the following way:
                          [ [route1, cost1], [route2, cost2], ..., [routeK, costK] ]
                      This function runs one OD-pair each time. For multiple OD-pairs,
                      it needs to be called multiple times.
v1.22 (25-Nov-2014) - Changed the type of edges' length attribute from int to float.
<new versions here>
'''

#!/usr/bin/python
import string
import argparse

# represents a node in the graph
class Node:
	def __init__(self, name):
		self.name = name	# name of the node
		self.dist = 1000000	# distance to this node from start node
		self.prev = None	# previous node to this node
		self.flag = 0		# access flag

# represents an edge in the graph
class Edge:
	def __init__(self, u, v, length):
		self.start = u
		self.end = v
		self.length = length

# read a text file and generate the graph according to declarations
def generateGraph(graph_file):
	V = []
	E = []
	fname = open(graph_file, "r")
	line = fname.readline()
	line = line[:-1]
	while line:
		taglist = string.split(line)
		if taglist[0] == 'node':
			V.append(Node(taglist[1]))
		elif taglist[0] == 'arc':
			E.append(Edge(taglist[1], taglist[2], float(taglist[3])))
		elif taglist[0] == 'edge':
			E.append(Edge(taglist[1], taglist[2], float(taglist[3])))
			E.append(Edge(taglist[2], taglist[1], float(taglist[3])))
		line = fname.readline()
		line = line[:-1]
	fname.close()
	return V, E

# reset graph's variables to default
def resetGraph(N, E):
	for node in N:
		node.dist = 1000000.0
		node.prev = None
		node.flag = 0

# returns the smallest node in N but not in S
def pickSmallestNode(N):
	minNode = None
	for node in N:
		if node.flag == 0:
			minNode = node
			break
	if minNode == None:
		return minNode
	for node in N:
		if node.flag == 0 and node.dist < minNode.dist:
			minNode = node
	return minNode

# returns the edges list of node u
def pickEdgesList(u, E):
	uv = []
	for edge in E:
		if edge.start == u.name:
			uv.append(edge)
	return uv

# Dijkstra's shortest path algorithm
def findShortestPath(N, E, origin, destination, ignoredEdges):
	
	#reset the graph (so as to discard information from previous runs)
	resetGraph(N, E)
	
	# set origin node distance to zero, and get destination node
	dest = None
	for node in N:
		if node.name == origin:
			node.dist = 0
		if node.name == destination:
			dest = node
	
	u = pickSmallestNode(N)
	while u != None:
		u.flag = 1
		uv = pickEdgesList(u, E)
		n = None
		for edge in uv:
			
			# avoid ignored edges
			if edge in ignoredEdges:
				continue
			
			# take the node n
			for node in N:
				if node.name == edge.end:
					n = node
					break
			if n.dist > u.dist + edge.length:
				n.dist = u.dist + edge.length
				n.prev = u
		
		u = pickSmallestNode(N)
		# stop when destination is reached
		if u == dest:
			break
	
	# generate the final path
	S = []
	u = dest
	while u.prev != None:
		S.insert(0,u)
		u = u.prev
	S.insert(0,u)
	
	return S

# print vertices and edges
def printGraph(N, E):
	print('vertices:')
	for node in N:
		previous = node.prev
		if previous == None:
			print(node.name, node.dist, previous)
		else:
			print(node.name, node.dist, previous.name)
	print('edges:')
	for edge in E:
		print(edge.start, edge.end, edge.length)

# print S path
def printPath(S, E):
	strout = ''
	for node in S:
		if strout != '':
			strout += ' - '
		strout += node.name
		
	print "%g = %s" % (calcPathLength(S, E), strout)

# generate a string from the path S in a specific format
def pathToString(S):
	strout = '['
	for i in xrange(0,len(S)-1):
		if i > 0:
			strout += ', '
		strout += '\'' + S[i].name + S[i+1].name + '\''
	return strout + ']'

# generate a list with the edges' names of a given route S
def pathToListOfString(S):
	lout = []
	for i in xrange(0,len(S)-1):
		lout.append(S[i].name + S[i+1].name)
	return lout

# get the directed edge from u to v
def getEdge(E, u, v):
	for edge in E:
		if edge.start == u and edge.end == v:
			return edge
	return None

def runKShortestPathsStep(V, E, origin, destination, k, A, B):
	# Step 0: iteration 1
	if k == 1:
		A.append(findShortestPath(V, E, origin, destination, []))
	
	# Step 1: iterations 2 to K
	else:
		lastPath = A[-1]
		for i in range(0, len(lastPath)-1):
			# Step I(a)
			spurNode = lastPath[i]
			rootPath = lastPath[0:i+1]
			toIgnore = []
			
			for path in A:
				if path[0:i+1] == rootPath:
					ed = getEdge(E, spurNode.name, path[i+1].name)
					toIgnore.append(ed)
			
			# Step I(b)
			spurPath = findShortestPath(V, E, spurNode.name, destination, toIgnore)
			if spurPath[0] != spurNode:
				continue
			
			# Step I(c)
			totalPath = rootPath + spurPath[1:]
			B.append(totalPath)
			
		#Step II
		bestInB = None
		bestInBlength = 999999999
		for path in B:
			length = calcPathLength(path, E)
			if length < bestInBlength:
				bestInBlength = length
				bestInB = path
		A.append(bestInB)
		while bestInB in B:
			B.remove(bestInB)
	
# Yen's K shortest loopless paths algorithm
def KShortestPaths(V, E, origin, destination, K):
	# the K shortest paths
	A = []
	
	# potential shortest paths
	B = []
	
	for k in xrange(1,K+1):
		try:
			runKShortestPathsStep(V, E, origin, destination, k, A, B)
		except:
			print 'Only %d paths were found!' % (k-1)
			break
		
	return A

# calculate path S's length
def calcPathLength(S, E):
	length = 0
	prev = None
	for node in S:
		if prev != None:
			length += getEdge(E, prev.name, node.name).length
		prev = node
	
	return length

# main procedure for many OD-pairs
def run(graph_file, OD_pairs, K):
	
	# read graph from file
	N, E = generateGraph(graph_file)
	
	#~ # find shortest path
	#~ S = findShortestPath(N, E, origin, destination, [])
	#~ printPath(S, E)
	
	#~ # find shortest path avoiding specific edges
	#~ S = findShortestPath(N, E, origin, destination, [E[1]])
	#~ printPath(S, E)
	
	# read list of OD-pairs
	OD = OD_pairs.split(';')
	for i in xrange(0,len(OD)):
		OD[i] = OD[i].split('|')
	
	# find K shortest paths of each OD-pair
	print 'ksptable = ['
	lastod = len(OD)-1
	for iod, (o, d) in enumerate(OD):
		# find K shortest paths for this specific OD-pair
		S = KShortestPaths(N, E, o, d, K)
		
		# print the result for this specific OD-pair
		print '\t[ # ' + o + d + ' flow'
		last = len(S)-1
		for i, path in enumerate(S):
			comma = ','
			if i == last:
				comma = ''
			print '\t\t' + pathToString(path) + comma + " # cost " + str(calcPathLength(path, E))
		comma = ','
		if iod == lastod:
			comma = ''
		print '\t]' + comma
	print ']'

# return a list with the K shortest paths for the given origin-destination pair
# this function was created to be called externally by another applications
def getKRoutes(graph_file, origin, destination, K):
	
	lout = []
	
	# read graph from file
	N, E = generateGraph(graph_file)
	
	# find K shortest paths for this specific OD-pair
	S = KShortestPaths(N, E, origin, destination, K)
	
	for path in S:
		# store the path (in list of strings format) and cost to the out list 
		lout.append([pathToListOfString(path), calcPathLength(path, E)])
		
	return lout
	
# initializing procedure
if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='KSP v1.22\nCompute the K shortest loopless paths between two nodes of a given graph, using Yen\'s algorithm [1].',
		epilog='GRAPH FILE FORMATTING INSTRUCTIONS' +
		'\nThe graph file supports three types of graphs\' entities: node, edge, arc. When creating the graph file, provide just one entity per line. \nUsage:'+
		'\n  node NAME\t  nodes of the graph' +
		'\n  edge N1 N2 W\t  create an undirected link between N1 and N2 with weight W' +
		'\n  arc N1 N2 W\t  create a directed link from N1 to N2 with weight W' +
		'\n\n  Example 1 - an undirected graph:' +
		'\n\tnode A' +
		'\n\tnode B' +
		'\n\tedge A B 10' +
		'\n\n  Example 2 - producing Example 1 with a directed graph:' +
		'\n\tnode A' +
		'\n\tnode B' +
		'\n\tarc A B 10' +
		'\n\tarc B A 10' +
		'\n\nREFERENCES' +
		'\n[1] Yen, J.Y.: Finding the k shortest loopless paths in a network. Management Science 17(11) (1971) 712-716.' +
		'\n\nAUTHOR' +
		'\nCreated in February 10, 2014, by Gabriel de Oliveira Ramos <goramos@inf.ufrgs.br>.',
		formatter_class=argparse.RawTextHelpFormatter)
	
	parser.add_argument('-f', dest='file', required=True,
						help='the graph file')
	parser.add_argument('-l', dest='OD_list', required=True,
						help='list of OD-pairs, in the format \'O|D;O|D;[and so on]\', where O are valid origin nodes, and D are valid destination nodes')
	parser.add_argument('-k', dest='K', type=int, required=True,
						help='number of shortest paths to find')
	args = parser.parse_args()
	
	graph_file = args.file
	OD_list = args.OD_list
	K = args.K
	
	run(graph_file, OD_list, K)
