import rospy
from topological_localization.msg import LineSegmentList

class LineFeatures:
    def line_extraction_callback(self, msg):
        self.line_list = msg.line_segments

    def __init__(self):
        self.line_list = []
        self.sub_lines = rospy.Subscriber('line_segments', LineSegmentList, self.line_extraction_callback)

    def print_lines(self):
        print 'Lines avaliable:'
        if self.line_list is not None:
            for line in self.line_list:
                print 'New line'
                print '\trho = ', line.radius
                print '\tangle = ', line.angle
                print '\tstart = ', line.start[0], line.start[1]
                print '\tend = ', line.end[0], line.end[1]
        print 'Next line'

    def run(self):
        self.print_lines()
