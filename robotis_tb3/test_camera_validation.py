"""Calibration diagnostics must pair matching corners across OpenCV array shapes."""
import unittest
import numpy as np
from validate_camera import reprojection_rmse


class ReprojectionTests(unittest.TestCase):
    def test_opencv_corner_layouts_compare_matching_points(self):
        points = np.array([[20., 30.], [150., 180.], [60., 100.]])
        self.assertEqual(reprojection_rmse(points[:, None, :], points), 0)
        self.assertEqual(reprojection_rmse(points, points[:, None, :]), 0)

    def test_known_pixel_displacement_and_invalid_pairs(self):
        points = np.array([[20., 30.], [150., 180.]])
        self.assertAlmostEqual(reprojection_rmse(points[:, None, :], points+[3, 4]), 5)
        with self.assertRaises(ValueError):
            reprojection_rmse(points, points[:1])
