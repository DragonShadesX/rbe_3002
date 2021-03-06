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

    def testMap1OutRangeHigh(self):
        self.assertTrue(self.map1.isOutOfRange(7,7))

    def testMap1OutRangeLow(self):
        self.assertTrue(self.map1.isOutOfRange(-1,-1))

    def testMap1CheckForValue(self):
        self.assertTrue(self.map1.checkForValue(0, 0, -1))
        self.assertTrue(self.map1.checkForValue(5, 6, -1))

    def testMap1FindFrontier(self):
        frontierCentroids = self.map1.findFrontiers()
        #print frontierCentroids
        self.assertItemsEqual(frontierCentroids, [(1,3), (2,1), (4,5)])
        self.assertEqual(len(frontierCentroids), 3)

    def testMap2NonCentroid(self):
        frontierCentroids = self.map2.findFrontiers()
        '''
         * Map2 should have only one fronteer because it eliminates the
         * frontiers that are surrounded by unknown space or walls.
         *
        '''
        self.assertItemsEqual(frontierCentroids, [(3,1)])
        self.assertEqual(len(frontierCentroids), 1)

if __name__ == '__main__':
    unittest.main()
