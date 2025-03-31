import pytest 
import numpy as np
from weap_cv_sensors.lidar import emulate_stream, async_get_scan, listen, circular_shift, calculate_angle_difference, get_distance, filter, detect_obstacles, cluster_points, create_occupancy_grid

def test_emulate_stream():
    """
    Test the emulate_stream function to ensure it generates a buffer of points.
    """
    buffer = emulate_stream()
    assert isinstance(buffer, list), "Buffer should be a list"
    assert len(buffer) == 1000, "Buffer should contain 1000 points"
    for point in buffer:
        assert isinstance(point, dict), "Each point should be a dictionary"
        assert 'x' in point and 'y' in point and 'z' in point and 'intensity' in point, "Each point should have x, y, z, and intensity keys"
        assert isinstance(point['x'], float), "x should be a float"
        assert isinstance(point['y'], float), "y should be a float"
        assert isinstance(point['z'], float), "z should be a float"
        assert isinstance(point['intensity'], int), "intensity should be an int"