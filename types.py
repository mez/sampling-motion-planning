from typing import NamedTuple

class Point(NamedTuple):
    x: float
    y: float


class Node:
    def __init__(self, location: Point):
        # location tells us where the node is in the configuration space
        self.location = location
        self.parent = None
        self.children = []