import rospy
import math
from geometry_msgs.msg import Point
from graph import Graph

def cartesian_distance(point1, point2):
    diffX = point2.x - point1.x
    diffY = point2.y - point1.y

    return math.sqrt(diffX ** 2 + diffY **2)

def vertex_distance(graph, vertex1, vertex2):
    point1 = graph.get_vertex(vertex1).get_point()
    point2 = graph.get_vertex(vertex2).get_point()

    return cartesian_distance(point1, point2)

def getMap():
    g = Graph()

    ## Add vertexes
    # upper layers
    # from left to right
    g.add_vertex('1')
    g.set_point('1', -27.5, 17.5)
    g.add_vertex('2')
    g.set_point('2', -13.5, 17.5)
    g.add_vertex('3')
    g.set_point('3', -1, 17.5)
    g.add_vertex('4')
    g.set_point('4', 8, 17.5)
    g.add_vertex('5')
    g.set_point('5', 19, 17.5)
    g.add_vertex('6')
    g.set_point('6', 31, 17.5)
    g.add_vertex('7')
    g.set_point('7', 38.5, 17.5)


    # middle layers
    # from left to right
    g.add_vertex('8')
    g.set_point('8', -27.5, 9)
    g.add_vertex('9')
    g.set_point('9', -1, 9)
    g.add_vertex('10')
    g.set_point('10', 19, 9)
    g.add_vertex('11')
    g.set_point('11', 38.5, 9)

    # down layer
    g.add_vertex('12')
    g.set_point('12', -27.5, 0)
    g.add_vertex('13')
    g.set_point('13', -13.5, 0)
    g.add_vertex('14')
    g.set_point('14', -1, 0)
    g.add_vertex('15')
    g.set_point('15', 8, 0)
    g.add_vertex('16')
    g.set_point('16', 19, 0)
    g.add_vertex('17')
    g.set_point('17', 31, 0)
    g.add_vertex('18')
    g.set_point('18', 38.5, 0)

    ## Make connections
    # upper layers connection
    g.add_edge('1', '2', vertex_distance(g, '1', '2'))
    g.add_edge('2', '3', vertex_distance(g, '2', '3'))
    g.add_edge('3', '4', vertex_distance(g, '3', '4'))
    g.add_edge('4', '5', vertex_distance(g, '4', '5'))
    g.add_edge('5', '6', vertex_distance(g, '5', '6'))
    g.add_edge('6', '7', vertex_distance(g, '6', '7'))

    # middle layers connection
    g.add_edge('1', '8', vertex_distance(g, '1', '8'))
    g.add_edge('8', '12', vertex_distance(g, '8', '12'))
    g.add_edge('3', '9', vertex_distance(g, '3', '9'))
    g.add_edge('9', '14', vertex_distance(g, '9', '14'))
    g.add_edge('5', '10', vertex_distance(g, '5', '10'))
    g.add_edge('10', '16', vertex_distance(g, '7', '11'))
    g.add_edge('7', '11', vertex_distance(g, '7', '11'))
    g.add_edge('11', '18', vertex_distance(g, '1', '18'))

    # botton layers connection
    g.add_edge('12', '13', vertex_distance(g, '12', '13'))
    g.add_edge('13', '14', vertex_distance(g, '13', '14'))
    g.add_edge('14', '15', vertex_distance(g, '14', '15'))
    g.add_edge('15', '16', vertex_distance(g, '15', '16'))
    g.add_edge('16', '17', vertex_distance(g, '17', '18'))
    g.add_edge('17', '18', vertex_distance(g, '17', '18'))

    return g
