#!/usr/bin/env python
from src.OccupancyMap_Object import OccupancyMap
from nav_msgs.msg import OccupancyGrid
import unittest

class TestOccupancyMapMethods(unittest.TestCase):
    def setUp(self):
        testGrid = OccupancyGrid()
        testGrid.info.height = 7
        testGrid.info.width = 6
        testGrid.data = [ -1,  -1,  -1,  -1,  -1,  -1,
                          -1, 100,   0,   0, 100,  -1,
                          -1, 100,   0,   0, 100,  -1,
                          -1,   0,   0,   0,   0,  -1,
                          -1, 100,   0,   0,   0,  -1,
                          -1, 100,   0,   0,   0,  -1,
                          -1,  -1,  -1,  -1,  -1,  -1]
        self.map = OccupancyMap(testGrid)

    def testOutRangeHigh(self):
        self.assertTrue(self.map.isOutOfRange(7,7))
    def testOutRangeLow(self):
        self.assertTrue(self.map.isOutOfRange(-1,-1))
    def testCheckForValue(self):
        self.assertTrue(self.map.checkForValue(0, 0, -1))
        self.assertTrue(self.map.checkForValue(5, 6, -1))

    def testFindFrontier(self):
        frontierCentroids = self.map.findFrontiers()
        self.assertItemsEqual(frontierCentroids, [(1,3), (2,1), (3,4)])
        self.assertEqual(len(frontierCentroids), 3)

if __name__ == '__main__':
    unittest.main()
