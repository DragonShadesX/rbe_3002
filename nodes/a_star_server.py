#!/usr/bin/env python

#BEGIN A STAR IMPLMEMENTATION
import collections
import sys
from itertools import ifilter

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

## END A STAR SOURCE IMPLEMENTATION


from rbe_3002.srv import *
import rospy
from nav_msgs.msg import OccupancyGrid

def map_callback(ret):
    global gridData
    #print ret
    width = ret.info.width
    height = ret.info.height
    gridData = GridWithWeights(width, height)
    gridData.walls = []
    #gridData.weights #If we ever choose to use weights here we can
    i = 0 # Element in the list
    for x in xrange(width):
        for y in xrange(height):
            if ret.data[i] == 100:
                gridData.walls.append((x, y))
            i += 1
    #print gridData.walls



def get_map():
    rospy.Subscriber("/map", OccupancyGrid, map_callback)




def a_star(req):
    rospy.loginfo("Request for a_star")
    while not rospy.is_shutdown():
        if not 'gridData' in globals():
            rospy.sleep(.01)
            break

    global gridData

    print req
    came_from, cost_so_far = a_star_search(gridData, req.startPoint, req.targetPoint)
    draw_grid(gridData, width=3, point_to=came_from, start=req.startPoint, goal= req.targetPoint)
    path = reconstruct_path(came_from, req.startPoint, req.targetPoint)
    path = tuple(path)
    pathx, pathy = zip(*path)
    print 'Path:'
    print path
    #print pathx
    #print pathy
    return AStarResponse(pathx, pathy)

def a_star_server():
    rospy.logdebug("Running A* Server")
    get_map()
    rospy.init_node('a_star_server')
    s = rospy.Service('a_star', AStar, a_star)
    rospy.spin()

if __name__ == '__main__':
    a_star_server()
