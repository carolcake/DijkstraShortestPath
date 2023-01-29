'''
Caroline Hsu - 01/29/2023
Using Priority Queue, I implemented Dijkstra's shortest path algorithm.
'''

# node class for name of the node
class Node:
    def __init__(self, name):
        self.name = name

# edge class for next node and length between the nodes
class Edge:
    def __init__(self, nextNode, length):
        self.nextNode = nextNode
        self.length = length

# graph class to create the graph/map of all the nodes
class Graph:
    def __init__(self):
        # create a set for the nodes (unordered)
        self.nodes = set()
        # create a dictionary for the edges
        self.edges = dict()

    # add node to the nodes
    def addNode(self, node):
        self.nodes.add(node)

    # add edge to the nodes
    def addEdge(self, beforeNode, nextNode, length):
        # declare new edge (where you need the next node and the length of it)
        edge = Edge(nextNode, length)
        # if the before node is already declared in edges
        if beforeNode.name in self.edges:
            beforeNode_edges = self.edges[beforeNode.name]
        #else if it's not already declared then make a new dictionary and add it
        else:
            self.edges[beforeNode.name] = dict()
            beforeNode_edges = self.edges[beforeNode.name]
        beforeNode_edges[nextNode.name] = edge

def findSmallestDist(unvisited, distance):
    # find smallest distance in unvisited nodes
    minNode = None
    # for every node in unvisited 
    for node in unvisited:
        if minNode == None:
            minNode = node
        # elif distance of the node is less than the distance of the minnode make node = minNode
        elif distance[node] < distance[minNode]:
            minNode = node
    # return minnode of the smallest distance
    return minNode

def dijkstra(graph, source):
    # declare max int for all the unknown distances
    maxInt = 999999999
    # empty dictionaries for distance and previous and create new set
    unvisited = set()
    distance = {}
    previous = {}

    for node in graph.nodes: 
        distance[node] = maxInt 
        previous[node] = maxInt 
        unvisited.add(node)  

    # distance from source to source
    distance[source] = 0
    
    # while there is still stuff in unvisited
    while unvisited:
        # node with the least distance selected first
        nodeLeastDistance = findSmallestDist(unvisited, distance)
        
        #remove the node with the least distance from the unvisited
        unvisited.remove(nodeLeastDistance)

        # if the node with the least distance is in the graph of edges
        if nodeLeastDistance.name in graph.edges:
            # go through the neighbors and find path
            for _, neighbor in graph.edges[nodeLeastDistance.name].items():
                alternateNode = distance[nodeLeastDistance] + neighbor.length
                if alternateNode < distance[neighbor.nextNode]:
                    # shorter path to goal node has been found
                    distance[neighbor.nextNode] = alternateNode
                    previous[neighbor.nextNode] = nodeLeastDistance
    # return the distance and previous node to main method 
    return distance, previous

# create the path of all the nodes from the first source node to the goal node (receives previous node and source node)
def path(prev, source):
    maxInt = 999999999
    previousNode = prev[source]
    path = [source.name]
    while previousNode != maxInt:
        # add the previous node's name to the path and swap 
        path.append(previousNode.name)
        temp = previousNode
        previousNode = prev[temp]
    
    # reverse the elements of the list so then it prints from the source node instead of from goal node
    path.reverse()
    return path

def main():
    # declare the graph of nodes and add all nodes from the example
    graph = Graph()
    node_a = Node("A")
    graph.addNode(node_a)
    node_b = Node("B")
    graph.addNode(node_b)
    node_c = Node("C")
    graph.addNode(node_c)
    node_d = Node("D")
    graph.addNode(node_d)
    node_f = Node("F")
    graph.addNode(node_f)
    node_g = Node("G")
    graph.addNode(node_g)
    node_h = Node("H")
    graph.addNode(node_h)
    node_j = Node("J")
    graph.addNode(node_j)
    
    # add all edges from the example given into the graph
    graph.addEdge(node_a, node_b, 4)
    graph.addEdge(node_a, node_c, 2)
    graph.addEdge(node_a, node_g, 7)
    graph.addEdge(node_b, node_d, 2)
    graph.addEdge(node_c, node_g, 3)
    graph.addEdge(node_c, node_f, 8)
    graph.addEdge(node_d, node_h, 6)
    graph.addEdge(node_d, node_g, 5)
    graph.addEdge(node_f, node_j, 3)
    graph.addEdge(node_g, node_j, 4)
    graph.addEdge(node_h, node_j, 2)
    
    #distance, previous node should be returned from the dijkstra's algo given graph and the source
    distance, previous = dijkstra(graph, node_a)

    # print quickest path with the distance including the correct path and the format (join makes them into one string)
    print("The quickest path from {} to {} is [{}] with a total distance of {}".format(node_a.name, node_j.name,
                                                                                 " to ".join(path(previous, node_j)),
                                                                                 str(distance[node_j])))
    
    
main()