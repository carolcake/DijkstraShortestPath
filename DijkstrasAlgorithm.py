# dijkstra's algorithm 
class Node:
    def __init__(self,name,edges):
        self.name = name
        self.edges = edges
        
def findSmallestDist(dataTable, unvisited):
    minVal = 999999999
    minNode = None
    for item in unvisited:
        for row in dataTable:
            if item.name == row[0]:
                if row[1] < minVal:
                    minVal = row[1]
                    minNode = item
    return minNode

def findNodeIndex(node, graph):
    index = 0
    for item in graph:
        if node == item:
            return index
        index += 1

# finds distance in the table and the node itself
def lookupDist(node, dataTable):
    for row in dataTable:
        if node.name == row[0]:
            return row[1]

def findUnvisitedIndex(node, unvisited):
    index = 0
    for item in unvisited:
        if node == unvisited:
            return index
    index += 1
    
def main():
    maxInt = 999999999
    A = Node("A", [0,4,-2,-1,-1,7,-1,-1])
    B = Node("B", [4,0,-1,2,-1,-1,-1,-1])
    C = Node("C", [2,-1,0,-1,8,3,-1,-1])
    D = Node("D", [-1,2,-1,0,-1,5,6,-1])
    F = Node("F", [-1,-1,8,-1,0,-1,-1,3])
    G = Node("G", [7,-1,3,5,-1,0,-1,4])
    H = Node("H", [-1,-1,-1,6,-1,-1,0,2])
    J = Node("J", [-1,-1,-1,-1,3,4,2,0])
    graph = [A,B,C,D,F,G,H,J]
    unvisited = [A,B,C,D,F,G,H,J]
    visited = []
    
    dataTable = [[A.name, 0, ""], [B.name, maxInt, ""], [C.name, maxInt, ""], [D.name, maxInt, ""],
                 [F.name, maxInt, ""], [G.name, maxInt, ""], [H.name, maxInt, ""], [J.name, maxInt, ""]]
    
    while unvisited != []:
        node = findSmallestDist(dataTable, unvisited) # finds the minimum distance node
        # creating an array with the unvisited neighbors
        unvisitedNeighbors = []
        index = 0
        # for each item in the nodes that are unvisited 
        for item in node.edges:
            if item > 0 and graph[index] in unvisited: # connected = relationship between the nodes
                unvisitedNeighbors.append(graph[index])
            index += 1
        
        # go through each neighbor
        for neighbor in unvisitedNeighbors:
            # find distance from current node to next neighbors
            distance = lookupDist(node, dataTable) + node.edges[findNodeIndex(neighbor, graph)]
            if distance < lookupDist(neighbor, dataTable):
                dataTable[findNodeIndex(neighbor, graph)][1] = distance
                dataTable[findNodeIndex(neighbor, graph)][2] = node.name
            
        # move node to visited
        unvisited.pop(findNodeIndex(node, unvisited))
        visited.append(node)
    
    # print table
    for row in dataTable:
        print(row)
main()