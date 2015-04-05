
# Sample code from http://www.redblobgames.com/pathfinding/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

class Graph(object):
    def __init__(self):
        self.edges = {}

    def neighbors(self, id):
        return self.edges[id]

example_graph = Graph()
example_graph.edges = {
    u'A': [u'B'],
    u'B': [u'A', u'C', u'D'],
    u'C': [u'A'],
    u'D': [u'E', u'A'],
    u'E': [u'B']
}

import collections
import sys
from itertools import ifilter

class Queue(object):
    def __init__(self):
        self.elements = collections.deque()

    def empty(self):
        return len(self.elements) == 0

    def put(self, x):
        self.elements.append(x)

    def get(self):
        return self.elements.popleft()

# utility functions for dealing with square grids
def from_id_width(id, **_3to2kwargs):
    width = _3to2kwargs['width']; del _3to2kwargs['width']
    return (id % width, id // width)

def draw_tile(graph, id, style, width):
    r = u"."
    if u'number' in style and id in style[u'number']: r = u"%d" % style[u'number'][id]
    if u'point_to' in style and style[u'point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style[u'point_to'][id]
        if x2 == x1 + 1: r = u"\u2192"
        if x2 == x1 - 1: r = u"\u2190"
        if y2 == y1 + 1: r = u"\u2193"
        if y2 == y1 - 1: r = u"\u2191"
    if u'start' in style and id == style[u'start']: r = u"A"
    if u'goal' in style and id == style[u'goal']: r = u"Z"
    if u'path' in style and id in style[u'path']: r = u"@"
    if id in graph.walls: r = u"#" * width
    return r

def draw_grid(graph, width=2, **style):
    for y in xrange(graph.height):
        for x in xrange(graph.width):
            print u"%%-%ds" % width % draw_tile(graph, (x, y), style, width),; sys.stdout.write(u"")
        print

# data from main article
DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]

class SquareGrid(object):
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id):
        return id not in self.walls

    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = ifilter(self.in_bounds, results)
        results = ifilter(self.passable, results)
        return results

class GridWithWeights(SquareGrid):
    def __init__(self, width, height):
        super(self.__class__, self).__init__(width, height)
        self.weights = {}

    def cost(self, a, b):
        return self.weights.get(b, 1)

diagram4 = GridWithWeights(10, 10)
diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
diagram4.weights = dict((loc, 5) for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6),
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6),
                                       (5, 7), (5, 8), (6, 2), (6, 3),
                                       (6, 4), (6, 5), (6, 6), (6, 7),
                                       (7, 3), (7, 4), (7, 5)])
draw_grid(diagram4)

import heapq

class PriorityQueue(object):
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

def dijkstra_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    return path

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far
