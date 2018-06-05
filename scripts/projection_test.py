import polygon2cog as p2c
import validation as v
import unittest
import numpy as np


class TestProjection(unittest.TestCase):
    def assertIdentity(self, pt2d, c2w, params):
        w2c = np.linalg.inv(c2w)
        pt3d = p2c.register(pt2d, c2w, params)
        pt2dd = v.project_point(np.array([pt3d]).T, w2c, params)
        self.assertAlmostEqual(pt2d[0], pt2dd[0][0])
        self.assertAlmostEqual(pt2d[1], pt2dd[1][0])
    def test_minimum(self):
        focallength = 1
        ccx = 0
        ccy = 0
        params = [focallength, 0, ccx, ccy]
        pt2d = np.array([0,0])
        c2w = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.assertIdentity(pt2d, c2w, params)
    def test_random(self):
        focallength = 2
        ccx = 30
        ccy = 40
        params = [focallength, 0, ccx, ccy]
        pt2d = np.array([-17,23])
        c2w = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.assertIdentity(pt2d, c2w, params)
    def test_random(self):
        focallength = 2
        ccx = 30
        ccy = 40
        params = [focallength, 0, ccx, ccy]
        pt2d = np.array([-17,23])
        c2w = np.array([[1, 0, 0, 199], [0, 1, 0, -12], [0, 0, 1, 5], [0, 0, 0, 1]])
        self.assertIdentity(pt2d, c2w, params)
    def test_random_rotation(self):
        focallength = 2
        ccx = 30
        ccy = 40
        params = [focallength, 0, ccx, ccy]
        pt2d = np.array([-17,23])
        c2w = np.array([
            [0.9106836, -0.2440169, 0.3333333, 199], 
            [0.3333333, 0.9106836, -0.2440168, -12], 
            [-0.2440169, 0.333333, 0.91068361, 5], 
            [0, 0, 0, 1]])
        self.assertIdentity(pt2d, c2w, params)

if __name__ == '__main__':
        unittest.main()
