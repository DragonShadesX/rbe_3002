#!/usr/bin/env python

''' Defines an object for manipulating a ocupency grid map'''
class OccupancyMap(object):
    ''' Defines a collection of frontier cells that form one frontier '''
    class Frontier_Collection(object):

        def __init__(self, x, y, map_instance):
            map_instance.frontierList.append(self)
            self.map_instance = map_instance
            self.elements = [(x, y)]

        def add(self, x, y):
            if len(self.elements) == 1: # If the lenght is 1 then we should be put as the next element in the list
                self.elements.append((x,y))
                return
            adjacents = self.map_instance.getAdjacentCellsList(x, y, diagonal=False)
            index = None
            for adjacent in adjacents:
                try:
                    index = self.elements.index(adjacent)
                    break
                except ValueError:
                    continue
            if index == None:
                adjacents = self.map_instance.getAdjacentCellsList(x, y)
                for adjacent in adjacents:
                    try:
                        index = self.elements.index(adjacent)
                        break
                    except ValueError:
                        continue
            #print "Index"
            #print index
            if index == 0:
                self.elements.insert(0, (x,y))
            elif index == None:
                raise ValueError("Can not add this value. Not adjacent to fontier")
            else:
                self.elements.append((x,y))

        '''
            Merges two Frontier_Collections together.
            Overwrites any previous refference to the old one
        '''
        def merge(self, aCollection):
            if isinstance(aCollection, OccupancyMap.Frontier_Collection):

                if self is aCollection:
                    # I don't know if this is possible but it is theoretically is
                    raise ValueError("Trying to merge self with self")
                notAdded = []
                for element in aCollection.elements:
                    try:
                        self.add(element[0], element[1])
                    except ValueError:
                        notAdded.append((element[0], element[1]))
                    self.map_instance.data[element[0]][element[1]] = self
                for element in notAdded:
                    self.add(element[0], element[1])
                    self.map_instance.data[element[0]][element[1]] = self
                #Remove this collection from the master list because it has been merged
                self.map_instance.frontierList.remove(aCollection)
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
        self.frontierList = []

    def _getFrontierCenters(self):
        #print "Frontier"
        centers = []
        for frontier in self.frontierList:
            # #print "Frontier"
            # #print frontier.elements
            # x = [p[0] for p in frontier.elements]
            # y = [p[1] for p in frontier.elements]
            # index = None
            # if len(frontier.elements) % 0 != 0:
            #     index = (len(frontier.elements)-1)/2
            # else:
            #print frontier.elements

            centroid = frontier.elements[(len(frontier.elements)-1)/2]
            centers.append(centroid)
        #print "Done"
        return centers

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
        if not self.checkForValue( x, y, 0, errorOutRange=True):
            return False
        checkCells = self.getAdjacentCellsList(x, y, diagonal=False)
        checkFrontierCount = 0
        checkWallCount = 0
        checkCellTotal = len(checkCells)
        for cell in checkCells:
            if self.checkForValue( cell[0], cell[1], -1):
                checkFrontierCount += 1
            if self.checkForValue( cell[0], cell[1], 100):
                checkWallCount += 1

        # Simple check to make sure that this logic never breaks
        assert (checkFrontierCount + checkWallCount) <= checkCellTotal

        # If this cell doesn't have another "known" cell next to it then assume that this is not a frontier
        # If there is at least one unknown cell in range then we have found an edge
        if (checkFrontierCount + checkWallCount) != checkCellTotal and checkFrontierCount > 0:
            return True
        return False

    ''' Checks if this cell is adjacent to another frontier'''
    def _getAdjacentFrontier(self, x, y):
        # List of cells to check for adjacent fronteer
        checkCells = self.getAdjacentCellsList(x, y, diagonal=False)
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
                            # Regardless the length of the list we need to add this cell to the first frontier
                            added = False
                            try:
                                adjacents[0].add(x,y)
                                added = True
                            except ValueError:
                                pass

                            # The list returned is greater than 0 then we are not alone
                            if len(adjacents) == 2:
                                # Since there is 2 or more frontiers next to us we have to do a multi merge
                                # This should be very rare
                                for i in xrange(len(adjacents)-1):
                                    adjacents[0].merge(adjacents[i+1])
                            elif len(adjacents) > 2:
                                raise "3 Adjacent Frontier"
                            if not added:
                                adjacents[0].add(x,y)

                            thisFrontier = adjacents[0]

                        else:
                            # We are alone. We need to create a new Frontier_Collection
                            # Hopefully someone else will join us soon!
                            thisFrontier = OccupancyMap.Frontier_Collection(x,y, self)

                        #Set this cell to thisFrontier
                        self.data[x][y] = thisFrontier
                    #Endif
                #Endif
            #Endfor
        #Endfor
        return self._getFrontierCenters()
