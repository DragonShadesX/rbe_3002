#!/usr/bin/env python

def calculate_slope(cord1, cord2):
    denominator = (cord1[0] - cord2[0])
    slope = None
    if (denominator != 0):
        slope = (cord1[1] - cord2[1])/denominator
    else:
        slope = float('inf')
    return slope

def path_to_waypoints(path):
    waypoints = []
    lastSlope = None
    for i in xrange(len(path)-1):
        slope = calculate_slope(path[i], path[i+1])
        if(lastSlope != slope):
            waypoints.append(path[i])
        lastSlope = slope
    waypoints.append(path[-1])
    return waypoints
