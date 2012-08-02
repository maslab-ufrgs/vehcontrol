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

from dicts import PriorityDict, DefaultDict
from decoratorclass import DecoratorClass

# Export ONLY the AStar class
__all__ = ['astar', 'dijkstra', 'AStar']


def astar(origin, destination, edge_cost_function, heuristic_distance_function):
    """Calculates the least-cost path from origin to destination.

    Applies edge_cost_function to obtain the cost of each edge, and
    also heuristic_distance_function to obtain the heuristic distance
    between each edge and the destination, using it to guide the search.
    """
    searcher = AStar(edge_cost_function, heuristic_distance_function)
    return searcher.search(origin, destination)

def dijkstra(origin, destination, edge_cost_function=None):
    """Calculates the least-cost path from origin to destination.

    Applies edge_cost_function to obtain the cost of each edge.
    """
    # Just an A* ignoring the heuristic
    return astar(origin, destination,
                 edge_cost_function or (lambda e: e.getLength()),
                 lambda a, b: 0.0)

class EdgeData(DecoratorClass):
    """Decorator class for Edges, adding information required for search."""

    # Constants for state
    UNVISITED = 0
    OPEN = 1
    CLOSED = 2

    def __init__(self, edge, state=UNVISITED, reaching_cost=0,
                 heuristic_cost=0, previous_edge=None):
        super(EdgeData, self).__init__(edge)
        self.__edge = edge

        self.state = state
        self.previous_edge = previous_edge

        self.__reaching_cost = float(reaching_cost)
        self.__heuristic_cost = float(heuristic_cost)
        self.__estimated_cost = self.__heuristic_cost + self.__reaching_cost

    @property
    def edge(self):
        return self.__edge

    @property
    def reaching_cost(self):
        """Float cost of best path reaching this edge."""
        return self.__reaching_cost
    @reaching_cost.setter
    def reaching_cost(self, val):
        self.__reaching_cost = val
        self.__estimated_cost = self.__heuristic_cost + val

    @property
    def heuristic_cost(self):
        """Float estimated cost from this edge to the destination."""
        return self._heuristic_cost
    @heuristic_cost.setter
    def heuristic_cost(self, val):
        self.__heuristic_cost = val
        self.__estimated_cost = self.__reaching_cost + val

    @property
    def estimated_cost(self):
        """Float estimated cost of the best path to destination through this."""
        return self.__estimated_cost

    def reconstruct_path(self):
        """Best path to this, obtained from the chain of previous_edges."""
        current = self
        result = deque()

        while current is not None:
            result.appendleft(current.edge)
            current = current.previous_edge

        return list(result)


class AStar(object):
    """Method object for performing heuristic graph search.

    The implementation is rather unusual: each edge is also
    treated as a node, which corresponds to its endpoint.

    The search is parameterized on the cost function for edges
    and on the heuristic distance.
    """

    def __init__(self, edge_cost_function, heuristic_distance_function):
        self.edge_cost = edge_cost_function
        self.heuristic_cost = (lambda edge:
                    heuristic_distance_function(edge, self.__destination))

    def search(self, origin, destination):
        """Performs a search from origin to destination.
        """

        # Initialize necessary structures/data
        self.__priority_queue = PriorityDict()
        self.__destination = EdgeData(destination)
        self.__edges = {destination: self.__destination}

        first = self.__edges.get(
            origin,
            EdgeData(origin, state=EdgeData.OPEN,
                     heuristic_cost=self.heuristic_cost(origin)))
        self.__priority_queue[first] = first.estimated_cost

        self.__edges = {origin: first, destination: self.__destination}

        # Main search body
        found_result = self.__search()

        # Reconstruct the result, if found
        if found_result:
            return self.__destination.reconstruct_path()
        else:
            return None

    def __search(self):
        """Main part of the search, returns True iff found a path.

        This method assumes that the instance is already initialized:
        destination set, origin on the priority queue.
        """
        # Bring common stuff into the namespace
        open_edges = self.__priority_queue
        destination = self.__destination

        # Keeps checking the open edges and visiting neighbors
        while len(open_edges) > 0:
            current_edge = open_edges.pop_smallest()
            current_edge.state = EdgeData.CLOSED

            # Ends search if found the destination
            if current_edge == destination:
                return True

            self.__visit_neighbors_of(current_edge)

        # Exhausted search and found no path
        return False

    def __visit_neighbors_of(self, edge):
        """Visits all neighbors of the current edge.

        This is supposed to happen after taking an
        edge from the priority queue and closing it.
        """
        # Bring some important values into the namespace
        all_edges = self.__edges
        open_edges = self.__priority_queue

        edge_cost = self.edge_cost
        heuristic_cost = self.heuristic_cost
        original_cost = edge.reaching_cost

        # Visit all neighbors
        for neighbor_edge in edge.getOutgoing():
            neighbor = all_edges.get(neighbor_edge, EdgeData(neighbor_edge))

            # Closed neighbors are NOT modified
            if neighbor.state == EdgeData.CLOSED:
                pass

            # Unvisited neighbors are initialized
            elif neighbor.state == EdgeData.UNVISITED:
                neighbor.state = EdgeData.OPEN
                neighbor.previous_edge = edge
                neighbor.reaching_cost = (edge_cost(neighbor_edge)
                                          + original_cost)
                neighbor.heuristic_cost = heuristic_cost(neighbor_edge)

                # Put the neighbor into the priority queue
                open_edges[neighbor] = neighbor.reaching_cost

            # Open neighbors are updated if the cost is lowered
            elif neighbor.state == EdgeData.OPEN:
                new_cost = (edge_cost(neighbor_edge)
                                    + original_cost)

                if new_cost < neighbor.reaching_cost:
                    neighbor.previous_edge = edge
                    neighbor.reaching_cost = new_cost
                    open_edges[neighbor] = neighbor.estimated_cost

            else:
                raise Exception("Invalid state for an EdgeData instance.")
