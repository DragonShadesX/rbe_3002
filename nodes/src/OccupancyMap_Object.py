#!/usr/bin/env python

''' Defines an object for manipulating a ocupency grid map'''
class OccupancyMap:
    ''' Defines a collection of frontier cells that form one frontier '''
    class Frontier_Collection(object):
        #Static list of these objects
        list = []
        @staticmethod
        def getFrontierCenters():
            centers = []
            for frontier in OccupancyMap.Frontier_Collection.list:
                #print "Frontier"
                #print frontier.elements
                x = [p[0] for p in frontier.elements]
                y = [p[1] for p in frontier.elements]
                centroid = (sum(x) / len(frontier.elements), sum(y) / len(frontier.elements))
                centers.append(centroid)
            return centers


        def __init__(self, x, y, map_instance):
            OccupancyMap.Frontier_Collection.list.append(self)
            self.map_instance = map_instance
            self.elements = [(x, y)]

        def add(self, x, y):
            self.elements.append((x,y))

        '''
            Merges two Frontier_Collections together.
            Overwrites any previous refference to the old one
        '''
        def merge(self, aCollection):
            if isinstance(aCollection, OccupancyMap.Frontier_Collection):

                if self is aCollection:
                    # I don't know if this is possible but it is theoreticalls
                    raise ValueError("Trying to merge self with self")

                for element in aCollection.elements:
                    self.add(element[0], element[1])
                    self.map_instance.data[element[0]][element[1]] = self
                #Remove this collection from the master list because it has been merged
                OccupancyMap.Frontier_Collection.list.remove(aCollection)
                # This should be the last time that we use this collection
                # If we use it again then there is a problem somewhere
                del aCollection
            else:
                raise TypeError("aCollection is of the wrong type")


    def __init__(self, mapData):
        self.width = mapData.info.width
        self.height = mapData.info.height
        data = mapData.data
        #self.data = [list(data[x:x+self.width]) for x in xrange(0, len(data), self.width)]
        self.data = [list(data[x::self.width]) for x in range( self.width)]

    def isOutOfRange(self, x, y):
        return (x < 0 or x >= self.width) or (y < 0 or y >= self.height)

    '''
     * Generates a list of adjacent cells for a given value.
     * If diagonals is true then include adjacently diagonals (default)
    '''
    def getAdjacentCellsList(self, x, y, diagonal=True):
        # Adds cells that are not diagonal
        cells = [(x, y+1), (x+1, y), (x, y-1), (x-1, y)]
        if diagonal:
            # Adds diagonal cells
            cells.extend([(x+1, y+1), (x+1, y-1), (x-1, y-1), (x-1, y+1)])
        return cells

    '''
     * Checks if the given cell has a specific check or passes the lambda.
     * Returns True if the check value is equal to the cell or the lambda functin returns true
     * errorOutRange Should an error be raised if the index is out of the range of the map (default False)
    '''
    def checkForValue(self, x, y, check, errorOutRange=False ):
        if self.isOutOfRange(x, y):
            if errorOutRange:
                raise IndexError(x + " or " + y + " out of range of map")
            else:
                return False
        else:
            if hasattr(check, '__call__'):
                return check(self.data[x][y])
            else:
                return self.data[x][y] == check

    ''' Checks to see if the given cell is on the fronteer'''
    def _isFrontier(self, x, y):
        #Checks above,
        # if not self.checkForValue( x, y, 0, errorOutRange=True):
        #     return False
        checkCells = self.getAdjacentCellsList(x, y, diagonal=False)
        for cell in checkCells:
            if self.checkForValue( cell[0], cell[1], -1):
                return True
        return False

    ''' Checks if this cell is adjacent to another frontier'''
    def _getAdjacentFrontier(self, x, y):
        # List of cells to check for adjacent fronteer
        checkCells = self.getAdjacentCellsList(x, y)
        adjacentFrontiers = []
        for cell in checkCells:
            if self.checkForValue( cell[0], cell[1], lambda x : (isinstance(x, OccupancyMap.Frontier_Collection) and x not in adjacentFrontiers)):
                adjacentFrontiers.append(self.data[cell[0]][cell[1]])
        return adjacentFrontiers

    def findFrontiers(self):
        #Iterate over all of the elements in the map
        for x in xrange(self.width):
            for y in xrange(self.height):
                #If the given map cell is a known location
                if self.checkForValue(x, y, 0, errorOutRange=True):
                    # Check if it is also a frontier
                    if self._isFrontier(x, y):
                        #We have found a fronteer
                        #Get the list of adjacent frontiers if there are any
                        adjacents = self._getAdjacentFrontier(x,y)
                        thisFrontier = None
                        if len(adjacents) > 0:
                            # The list returned is greater than 0 then we are not alone
                            if len(adjacents) >= 2:
                                # Since there is 2 or more frontiers next to us we have to do a multi merge
                                # This should be very rare
                                for i in xrange(len(adjacents)-1):
                                    adjacents[0].merge(adjacents[i+1])

                            # Regardless the length of the list we need to add this cell to the first frontier
                            adjacents[0].add(x,y)
                            thisFrontier = adjacents[0]

                        else:
                            # We are alone. We need to create a new Frontier_Collection
                            # Hopefully someone else will join us soon!
                            thisFrontier = OccupancyMap.Frontier_Collection(x,y, self)

                        #Set this cell to thisFrontier
                        self.data[x][y] = thisFrontier
        return OccupancyMap.Frontier_Collection.getFrontierCenters()
