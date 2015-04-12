#!/usr/bin/env python

#Takes the rediculously long list and breaks it up into chuncks of width
def chunk(data, width, height):
    return [list(data[x:x+width]) for x in xrange(0, len(data), width)]
