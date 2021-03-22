import sys
sys.path.append('../scripts')

import unittest
import ekf

class TestPropagation(unittest.TestCase):

    def test_propagation(self):
        ekf.propagate()
        a = 1

if __name__ == '__main__':
    unittest.main()