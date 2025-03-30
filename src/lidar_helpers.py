import time
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