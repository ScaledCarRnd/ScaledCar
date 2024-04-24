import unittest
from scripts.cam_functions import *

class TestWidthFinder(unittest.TestCase):
    def test_width_finder_when_both_values_below_threshold(self):
        result = widthFinder(200, 240)
        self.assertEqual(result, 10, "Expected width not returned.")

if __name__ == '__main__':
    unittest.main()
