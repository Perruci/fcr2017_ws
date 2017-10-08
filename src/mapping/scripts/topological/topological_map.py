import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

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
    def get_pose_callback(self, pose_data):
        if pose_data.size() != 2:
            print 'POSE_SUB: wrong number of points recieved'
        else:
            self.target_poses = pose_array_data
            self.recieved_target = True

    def __init__(self):
        self.graph = initialize_map()
        self.best_path = get_shortest_path(self.graph, '1', '18')
        # ROS setup
        self.sub_pose = rospy.Subscriber('topological/where_to', Pose, self.get_pose_callback)
        self.pub_pose = rospy.Publisher('topological/best_path/poses', PoseArray, queue_size=10)
        self.pub_id = rospy.Publisher('topological/best_path/ids', String, queue_size=10)
        self.target_poses = set()
        self.recieved_target = False

    def run(self):
        # generate messages
        msg_string = get_id_msg(self.best_path)
        self.pub_id.publish(msg_string)
        rospy.loginfo('paths id published')

        msg_pose = generate_poses_msg(self.best_path)
        self.pub_pose.publish(msg_pose)
        rospy.loginfo('paths points published')
