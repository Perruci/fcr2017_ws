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

    print 'Graph data:'
    for v in g:
        for w in v.get_connections():
            vid = v.get_id()
            wid = w.get_id()
            print '( %s , %s, %3d)'  % ( vid, wid, v.get_weight(w))
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

if __name__ == '__main__':

    # ROS setup
    pub_pose = rospy.Publisher('topological/best_path/poses', PoseArray, queue_size=10)
    pub_id = rospy.Publisher('topological/best_path/ids', String, queue_size=10)
    rospy.init_node('topological')
    r = rospy.Rate(1) # 1hz

    # Map setup
    cic_graph = initialize_map()
    best_path = get_shortest_path(cic_graph, '1', '18')

    while not rospy.is_shutdown():
        # generate messages
        msg_string = get_id_msg(best_path)
        pub_id.publish(msg_string)

        msg_pose = generate_poses_msg(best_path)
        pub_pose.publish(msg_pose)
        r.sleep()
