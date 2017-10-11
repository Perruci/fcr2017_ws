import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point

class TopologicalMonitor:
    ''' Topological Map and Grid Map interface '''
    def id_callback(self, msg):
        self.current_id = msg.data
        if not self.id_set:
            self.id_set = True

    def region_callback(self, poses_msg):
        if len(poses_msg.poses) != 2:
            print 'recieved more current region poses than expected'
            return
        pose_list = poses_msg.poses
        self.regionPt1 = pose_list[0].position
        self.regionPt2 = pose_list[1].position
        if not self.region_set:
            self.region_set = True

    def __init__(self):
        self.id_set = False
        self.region_set = False
        self.sub_current_id = rospy.Subscriber('topological/current/id', String, self.id_callback)
        self.sub_current_region = rospy.Subscriber('topological/current/region', PoseArray, self.region_callback)
        self.regionPt1 = Point()
        self.regionPt2 = Point()

    def get_id(self):
        if self.id_set:
            return self.current_id
        else:
            return '0'

    def get_region(self):
        if self.region_set:
            return self.regionPt1, self.regionPt2
        else:
            return None, None
