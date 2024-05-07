import unittest
from camfunctions import *

class TestMetricFinder(unittest.TestCase):

    def setUp(self):
        self.widtherr = 'Expected width not returned.'

    # Test for when both bl and br are in the same range
    def test_both_pixels_in_same_range(self):
        # 0 - 245
        self.assertEqual(widthFinder(200, 240), 10, self.widtherr)
        self.assertEqual(widthFinder(0, 245), 10, self.widtherr)

        # 246 - 705
        self.assertEqual(widthFinder(246, 705), 10, self.widtherr)
        self.assertEqual(widthFinder(300, 600), 10, self.widtherr)

        # 706 - 1155
        self.assertEqual(widthFinder(706, 1155), 10, self.widtherr)
        self.assertEqual(widthFinder(800, 1100), 10, self.widtherr)

        # 1156 - 1280
        self.assertEqual(widthFinder(1156, 1280), 4, self.widtherr)
        self.assertEqual(widthFinder(1200, 1250), 4, self.widtherr)

    # Test for when only bl is below 246 pixels
    def test_only_bl_below_246(self):
        self.assertEqual(widthFinder(200, 705), 20, self.widtherr)
        self.assertEqual(widthFinder(200, 1155), 30, self.widtherr)
        self.assertEqual(widthFinder(200, 1280), 34, self.widtherr)
        self.assertEqual(widthFinder(245, 1279), 34, self.widtherr)

    # Test for when bl is above 245 but below 706 while br is above 705
    def test_bl_above_245_below_706_br_above_705(self):
        self.assertEqual(widthFinder(500, 800), 20, self.widtherr)
        self.assertEqual(widthFinder(500, 1200), 24, self.widtherr)
        self.assertEqual(widthFinder(705, 706), 20, self.widtherr)
        self.assertEqual(widthFinder(705, 1156), 24, self.widtherr)

    # Test for when bl is above 705 and below 1156 and br is above 1155
    def test_bl_above_705_below_1156_br_above_1155(self):
        self.assertEqual(widthFinder(800,1200), 14, self.widtherr)
        self.assertEqual(widthFinder(706, 1156), 14, self.widtherr)


    # Test for distances
    # Test cases for distanceFinder function
    def test_distanceFinder(self):
        # Test case 1: bp > 392
        self.assertEqual(distanceFinder(400), 5,"Test case 1 failed")
        
        # Test case 2: 207 < bp <= 392
        self.assertEqual(distanceFinder(300), 10,"Test case 2 failed")

        # Test case 3: 121 < bp <= 207
        self.assertEqual(distanceFinder(150), 20,"Test case 3 failed")
        
        # Test case 4: 72 < bp <= 121
        self.assertEqual(distanceFinder(100), 30,"Test case 4 failed")
        
        # Test case 5: 40 < bp <= 72
        self.assertEqual(distanceFinder(50), 40,"Test case 5 failed")
        
        # Test case 6: 22 < bp <= 40
        self.assertEqual(distanceFinder(30), 50,"Test case 6 failed")
        
        # Test case 7: bp <= 22
        self.assertEqual(distanceFinder(10), 60,"Test case 7 failed")
        

if __name__ == '__main__':
    unittest.main()
