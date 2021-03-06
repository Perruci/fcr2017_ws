import sys
import rospy
from geometry_msgs.msg import Point

# thanks to http://www.bogotobogo.com/python/python_Dijkstras_Shortest_Path_Algorithm.php

class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxint
        # Mark all nodes unvisited
        self.visited = False
        # Predecessor
        self.previous = None
        # Central Point
        self.point = Point
        # Region bounded points
        self.regionPt1 = Point
        self.regionPt2 = Point

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def set_unvisited(self):
        self.visited = False

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

    # Points functions
    def set_point(self, x, y):
        self.point = Point(x, y, 0)

    def get_point(self):
        return self.point

    def get_pointX(self):
        return self.point.x

    def get_pointY(self):
        return self.point.y

    # Region functions
    def set_region(self, point1, point2):
        self.regionPt1 = point1
        self.regionPt2 = point2

    def get_region(self):
        return (self.regionPt1,self.regionPt2)

    # definido sempre com P1.x < P2.x e P1.y > P2.y
    def is_inside(self, point):
        if point.x >= self.regionPt1.x and point.x <= self.regionPt2.x:
            if point.y <= self.regionPt1.y and point.y >= self.regionPt2.y:
                return True
        return False

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

    def set_point(self, vertex, x, y):
        if vertex not in self.vert_dict:
            self.add_vertex(vertex)
        self.vert_dict[vertex].set_point(x, y)

    def set_region(self, vertex, point1, point2):
        if vertex not in self.vert_dict:
            self.add_vertex(vertex)
        self.vert_dict[vertex].set_region(point1, point2)

    def get_region(self, vertex):
        if vertex in self.vert_dict:
            return self.vert_dict[vertex].get_region()
        else:
            return None

    def is_inside(self, vertex, point):
        if self.vert_dict[vertex].is_inside(point):
            return True
        else:
            return False

    def reset_visited(self):
        for key, vertex in self.vert_dict.items():
            vertex.set_unvisited()
