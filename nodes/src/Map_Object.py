
class Map:
    def __init__(self, mapData):
        self.width = mapData.info.width
        self.height = mapData.info.height
        data = mapData.data
        self.data = [list(data[x:x+self.width]) for x in xrange(0, len(data), self.width)]

    def checkOutOfRange(self, x, y):
        return (x < 0 or x >= self.height) or (y < 0 or y >= self.width)

    '''
     * Generates a list of adjacent cells for a given value.
     * If diagonals is true then include adjacently diagonals (default)
    '''
    def getAdjacentCellsList(self, x, y, diagonal=True):
        cells = [(x, y+1), (x+1, y), (x, y-1), (x-1, y)]
        if diagonal:
            cells.extend([(x+1, y+1), (x+1, y-1), (x-1, y-1), (x-1, y+1)])
        return cells

    '''
     * Checks if the given cell has a specific value.
     * Returns True if it contains the value
     * errorOutRange Should an error be raised if the index is out of the range of the map (default False)
    '''
    def checkForValue(self, x, y, value, errorOutRange=False ):
        if checkOutOfRange(x, y):
            if errorOutRange:
                raise IndexError(x + " or " + y + " out of range of map")
            else:
                return False
        else:
            return self.data[x][y] == value


    ''' Checks to see if the given cell is on the fronteer'''
    def _isFrontier(self, x, y):
        #Checks above,
        checkCells = getAdjacentCellsList(x, y, diagonal=False)
        for cell in checkCells:
            if checkForValue( cell[0], cell[1], -1):
                return True
        return False

    ''' Checks if this cell is adjacent to another frontier'''
    def _isAdjacentFrontier(self, x, y):
        # List of cells to check for adjacent fronteer
        checkCells = getAdjacentCellsList(x, y)
        for cell in checkCells:
            if checkForValue( cell[0], cell[1], 2):
                return True
        return False

    def find_frontiers(self):
        for y in xrange(self.width):
            for x in xrange(self.height):
                if checkForValue(x, y, 1, errorOutRange=True):
                    if _isFrontier(x, y):
                        #We have found a fronteer
                        self.data[x][y]=1
