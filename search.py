
"""Implementation of the graph search algorithms over SUMO networks.

The searched graph is defined by an instance of sumolib.Net,
and the graph nodes and edges are actually network edges and
connections, respectively.

First contains a decorator class for edges that contains extra
information about cost and state, used for serching. Also contains
a pool for instances of this class.

Also contains method-object classes for the algorithms, taking
as parameters a function for obtaining the cost of each edge and
for calculating heuristics.
"""
from collections import deque
from math import sqrt

from prioritydict import priority_dict
from decoratorclass import DecoratorClass

# Export ONLY the AStar class
__all__ = ['astar', 'dijkstra', 'AStar']


def astar(origin, destination, edgeCostFunction, heuristicDistanceFunction):
    searcher = AStar(edgeCostFunction, heuristicDistanceFunction)
    return searcher.search(origin, destination)

def dijkstra(origin, destination, edgeCostFunction=None):
    # Just an A* ignoring the heuristic
    return astar(origin, destination, 
                 edgeCostFunction or (lambda e: e.getLength()),
                 lambda a, b: 0.0)
    return searcher.search(origin, destination)

class EdgeData(DecoratorClass):
    """Decorator class for Edges, adding information required for search."""

    # Constants for state
    UNVISITED = 0
    OPEN = 1
    CLOSED = 2

    def __init__(self, edge, state=UNVISITED, reachingCost=0,
                 heuristicCost=0, previousEdge=None):
        super(EdgeData, self).__init__(edge)
        self.__edge = edge

        self.state = state
        self.previousEdge = previousEdge

        self.__reachingCost = float(reachingCost)
        self.__heuristicCost = float(heuristicCost)
        self.__estimatedCost = self.__heuristicCost + self.__reachingCost

    @property
    def edge(self):
        return self.__edge

    @property
    def reachingCost(self):
        """Float cost of best path reaching this edge."""
        return self.__reachingCost
    @reachingCost.setter
    def reachingCost(self, val):
        self.__reachingCost = val
        self.__estimatedCost = self.__heuristicCost + val

    @property
    def heuristicCost(self):
        """Float estimated cost from this edge to the destination."""
        return self._heuristicCost
    @heuristicCost.setter
    def heuristicCost(self, val):
        self.__heuristicCost = val
        self.__estimatedCost = self.__reachingCost + val

    @property
    def estimatedCost(self):
        """Float estimated cost of the best path to destination through this."""
        return self.__estimatedCost

    def reconstructPath(self):
        """Best path to this, obtained from the chain of previousEdges."""
        current = self
        result = deque()

        while current is not None:
            result.appendleft(current.edge)
            current = current.previousEdge

        return list(result)


class AStar(object):
    """Method object for performing heuristic graph search.

    The implementation is rather unusual: each edge is also
    treated as a node, which corresponds to its endpoint.

    The search is parameterized on the cost function for edges
    and on the heuristic distance.
    """

    def __init__(self, edgeCostFunction, heuristicDistanceFunction):
        self.edgeCost = edgeCostFunction
        self.heuristicCost = lambda edge: heuristicDistanceFunction(edge, self.__destination)

    def search(self, origin, destination):
        """Performs a search from origin to destination.
        """

        # Initialize necessary structures/data
        self.__priorityQueue = priority_dict()
        self.__destination = EdgeData(destination)
        self.__edges = {destination: self.__destination}

        first = self.__edges.get(
            origin,
            EdgeData(origin, state=EdgeData.OPEN,
                     heuristicCost=self.heuristicCost(origin)))
        self.__priorityQueue[first] = first.estimatedCost

        self.__edges = {origin: first, destination: self.__destination}

        # Main search body
        foundResult = self.__search()

        # Reconstruct the result, if found
        if foundResult:
            return self.__destination.reconstructPath()
        else:
            return None

    def __search(self):
        """Main part of the search, returns True iff found a path.

        This method assumes that the instance is already initialized:
        destination set, origin on the priority queue.
        """
        # Bring common stuff into the namespace
        openEdges = self.__priorityQueue
        destination = self.__destination

        # Keeps checking the open edges and visiting neighbors
        while len(openEdges) > 0:
            currentEdge = openEdges.pop_smallest()
            currentEdge.state = EdgeData.CLOSED

            # Ends search if found the destination
            if currentEdge == destination:
                return True

            self.__visitNeighborsOf(currentEdge)

        # Exhausted search and found no path
        return False

    def __visitNeighborsOf(self, edge):
        """Visits all neighbors of the current edge.

        This is supposed to happen after taking an
        edge from the priority queue and closing it.
        """
        # Bring some important values into the namespace
        allEdges = self.__edges
        openEdges = self.__priorityQueue

        edgeCost = self.edgeCost
        heuristicCost = self.heuristicCost
        originalCost = edge.reachingCost

        # Visit all neighbors
        for neighborEdge in edge.getOutgoing():
            neighbor = allEdges.get(neighborEdge, EdgeData(neighborEdge))

            # Closed neighbors are NOT modified
            if neighbor.state == EdgeData.CLOSED:
                pass

            # Unvisited neighbors are initialized
            elif neighbor.state == EdgeData.UNVISITED:
                neighbor.state = EdgeData.OPEN
                neighbor.previousEdge = edge
                neighbor.reachingCost = ( edgeCost(neighborEdge)
                                          + originalCost )
                neighbor.heuristicCost = heuristicCost(neighborEdge)

                # Put the neighbor into the priority queue
                openEdges[neighbor] = neighbor.reachingCost

            # Open neighbors are updated if the cost is lowered
            elif neighbor.state == EdgeData.OPEN:
                newReachingCost = ( edgeCost(neighborEdge)
                                    + originalCost )

                if newReachingCost < neighbor.reachingCost:
                    neighbor.previousEdge = edge
                    neighbor.reachingCost = newReachingCost
                    openEdges[neighbor] = neighbor.estimatedCost

            else:
                raise Exception("Invalid state for an EdgeData instance.")
