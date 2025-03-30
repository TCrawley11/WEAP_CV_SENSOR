import time
import math
from typing import List
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def emulate_stream():
    """
    This function emulates a stream of LiDAR data by generating random points.
    It creates a list of dictionaries, each representing a point with x, y, z coordinates and intensity.
    """
    import random

    # Create a buffer to store points
    buffer = []

    # Generate a list of 1000 random points and add them to the buffer
    for _ in range(1000):
        point = {
            'x': random.uniform(-10.0, 10.0),
            'y': random.uniform(-10.0, 10.0),
            'z': random.uniform(-10.0, 10.0),
            'intensity': random.randint(0, 255)
        }
        buffer.append(point)

    return buffer

def circular_shift(data: List[float]) -> List[float]:
    """
    Perform a circular shift on the input list.
    The first element moves to the end of the list.
    """
    if not data:
        return data
    return data[1:] + [data[0]]

def calculate_angle_difference(start_angle: int, end_angle: int) -> float:
    """
    Calculate the angle in radians from start_angle to end_angle.
    The angles are given in hundredths of a degree.
    """
    start_angle_rad = math.radians(start_angle / 100.0)
    end_angle_rad = math.radians(end_angle / 100.0)
    return end_angle_rad - start_angle_rad

def get_distance(data: List[float]) -> float:
    """
    Calculate the distance from the data list.
    The first element is the distance, the second is the strength.
    """
    if len(data) < 2:
        raise ValueError("Data list must contain at least two elements.")
    distance = data[0]
    strength = data[1]
    return distance, strength

def listen(time: int):
    for i in range(time):
        # Simulate listening for data
        print(f"Listening... {i + 1}/{time}")
        async_get_scan() # call the simulated scan function
        time.sleep(0.1) # simulate small delay

def draw_bitmap():
    fig = plt.figure()

def async_get_scan():
    """
    Main function used to get the LiDAR scan data and call other helper functions.
    """
    emulate_stream()