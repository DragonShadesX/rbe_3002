#!/usr/bin/env python
from src.OccupancyMap_Object import OccupancyMap
from nav_msgs.msg import OccupancyGrid
import unittest

class TestOccupancyMapMethods(unittest.TestCase):
    def setUp(self):
        testGrid1 = OccupancyGrid()
        testGrid1.info.height = 7
        testGrid1.info.width = 6
        testGrid1.data = [ -1,  -1,  -1,  -1,  -1,  -1,
                           -1, 100,   0,   0, 100,  -1,
                           -1, 100,   0,   0, 100,  -1,
                           -1,   0,   0,   0,   0,  -1,
                           -1, 100,   0,   0,   0,  -1,
                           -1, 100,   0,   0,   0,  -1,
                           -1,  -1,  -1,  -1,  -1,  -1]
        self.map1 = OccupancyMap(testGrid1)

        testGrid2 = OccupancyGrid()
        testGrid2.info.height = 7
        testGrid2.info.width = 6
        testGrid2.data = [ -1,  -1,  -1,   0,  -1,  -1,
                           -1,   0,  -1,   0, 100,  -1,
                           -1,  -1,  -1,   0, 100,  -1,
                           -1, 100, 100,   0, 100,  -1,
                           -1, 100,   0, 100, 100,  -1,
                           -1, 100, 100, 100,   0,  -1,
                           -1,  -1,  -1,  -1,  -1,  -1]
        self.map2 = OccupancyMap(testGrid2)

    def testOutRangeHigh(self):
        self.assertTrue(self.map1.isOutOfRange(7,7))
        
    def testOutRangeLow(self):
        self.assertTrue(self.map1.isOutOfRange(-1,-1))

    def testCheckForValue(self):
        self.assertTrue(self.map1.checkForValue(0, 0, -1))
        self.assertTrue(self.map1.checkForValue(5, 6, -1))

    def testFindFrontier(self):
        frontierCentroids = self.map1.findFrontiers()
        self.assertItemsEqual(frontierCentroids, [(1,3), (2,1), (3,4)])
        self.assertEqual(len(frontierCentroids), 3)

    def testNonCentroid(self):
        frontierCentroids = self.map2.findFrontiers()
        self.assertItemsEqual(frontierCentroids, [(3,1)])
        self.assertEqual(len(frontierCentroids), 1)

if __name__ == '__main__':
    unittest.main()
