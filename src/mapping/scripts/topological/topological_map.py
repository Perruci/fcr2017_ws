import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

import dijkstra as dj
from graph import Graph
from cic_map import getMap

# Mapping functions
def initialize_map():
    g = getMap()

    # print 'Graph data:'
    for v in g:
        for w in v.get_connections():
            vid = v.get_id()
            wid = w.get_id()
            # print '( %s , %s, %3d)'  % ( vid, wid, v.get_weight(w))
    print 'Graph setup complete'
    return g

def get_vertex_from_point(g, point):
    # get vertex names
    keys = g.get_vertices()
    for v_id in keys:
        if g.is_inside(v_id, point):
            return v_id
    print 'target point not found on vertices'
    return '0'

def get_shortest_path(g, initial, target):
    dj.dijkstra(g, g.get_vertex(initial), g.get_vertex(target))
    target = g.get_vertex(target)
    path = [target]
    dj.shortest(target, path)
    return path

def print_path(path):
    print 'The shortest path : '
    for node in path:
        print 'Node [', node.get_id(), '] (', node.get_pointX(), ', ', node.get_pointY(), ')'

# ROS functions
def get_id_msg(path):
    msg = String()
    for node in path:
        msg.data += node.get_id() + ' '
    return msg

def get_pose(point):
    pose = Pose()
    pose.position = point
    return pose

def initialize_pose_array():
    pose_array = PoseArray()
    return pose_array

def generate_poses_msg(path):
    pose_array = initialize_pose_array()
    for node in path:
        pose_array.poses.append(get_pose(node.get_point()))

    return pose_array


class TopologicalMap:
    def get_origin_callback(self, odometry_data):
        # get pose from odometry message
        self.origin_pose = odometry_data.pose.pose
        print 'recieved origin pose: (', self.origin_pose.position.x, ', ',self.origin_pose.position.y, ')'
        self.origin_id = get_vertex_from_point(self.graph, self.origin_pose.position)
        print 'origin id found: ', self.origin_id
        if self.origin_id != '0':
            self.recieved_origin = True


    def get_target_callback(self, pose_data):
        self.target_pose = pose_data
        print 'recieved target pose: (', self.target_pose.position.x, ', ',self.target_pose.position.y, ')'
        self.target_id = get_vertex_from_point(self.graph, self.target_pose.position)
        print 'target id found: ', self.target_id
        if self.target_id != '0' and self.recieved_origin:
            self.best_path = get_shortest_path(self.graph, self.origin_id, self.target_id)
            self.recieved_target = True

    def __init__(self):
        self.graph = initialize_map()
        # ROS setup
        self.sub_origin = rospy.Subscriber('/pose', Odometry, self.get_origin_callback)
        self.sub_target = rospy.Subscriber('topological/where_to', Pose, self.get_target_callback)
        self.pub_pose = rospy.Publisher('topological/best_path/poses', PoseArray, queue_size=10)
        self.pub_id = rospy.Publisher('topological/best_path/ids', String, queue_size=10)
        self.origin_pose = Pose()
        self.origin_id = '0'
        self.recieved_origin = False
        self.target_pose = Pose()
        self.target_id = '0'
        self.recieved_target = False

    def run(self):
        if self.recieved_origin and self.recieved_target:
            # generate messages
            msg_string = get_id_msg(self.best_path)
            self.pub_id.publish(msg_string)
            rospy.loginfo('paths id published')

            msg_pose = generate_poses_msg(self.best_path)
            self.pub_pose.publish(msg_pose)
            rospy.loginfo('paths points published')

            self.recieved_target = False
        else:
            rospy.loginfo('waiting for directions')
