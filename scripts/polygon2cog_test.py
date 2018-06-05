import polygon2cog as p2c
import unittest
import numpy as np

def normalize(v):
    return v / np.linalg.norm(v)

class TestComputeNearestPoints(unittest.TestCase):
    def test_general_close(self):
        ray1 = (np.array([0,0,3]), normalize(np.array([3,2,-1]) - np.array([0,0,3])))
        ray2 = (np.array([-3,0,0]), normalize(np.array([4,3,1]) - np.array([-3,0,0])))
        EFg = np.array([1.850253807, 1.233502538, 0.5329949239])
        EFh = np.array([1.530456853, 1.941624365, 0.6472081218])
        F = p2c.computeNearestPoints(ray1, ray2)
        Fg = F[0]
        Fh = F[1]
        places = 7
        
        self.assertAlmostEqual(Fg[0], EFg[0], places)
        self.assertAlmostEqual(Fg[1], EFg[1], places)
        self.assertAlmostEqual(Fg[2], EFg[2], places)
        self.assertAlmostEqual(Fh[0], EFh[0], places)
        self.assertAlmostEqual(Fh[1], EFh[1], places)
        self.assertAlmostEqual(Fh[2], EFh[2], places)

class TestFindRayCenter(unittest.TestCase):
    def test_general_close_2_rays(self):
        ray1 = (np.array([0,0,3]), normalize(np.array([3,2,-1]) - np.array([0,0,3])))
        ray2 = (np.array([-3,0,0]), normalize(np.array([4,3,1]) - np.array([-3,0,0])))
        EFg = np.array([1.850253807, 1.233502538, 0.5329949239])
        EFh = np.array([1.530456853, 1.941624365, 0.6472081218])
        Ec = (EFg + EFh)/2
        places = 7
        
        c = p2c.findRayCenter([ray1, ray2])

        self.assertAlmostEqual(c[0], Ec[0], places)
        self.assertAlmostEqual(c[1], Ec[1], places)
        self.assertAlmostEqual(c[2], Ec[2], places)


if __name__ == '__main__':
        unittest.main()
