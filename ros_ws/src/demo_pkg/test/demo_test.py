#!/usr/bin/env python3

import unittest
import rostest

class DemoTest(unittest.TestCase):
    def test_dummy(self):
        """This is a dummy test that always passes"""
        self.assertTrue(True)

if __name__ == '__main__':
    rostest.rosrun('demo_pkg', 'demo_test', DemoTest)
