import rospy
import math
import numpy as np
from topological_localization.msg import LineSegmentList

def line_length(start, end):
    ''' Compute line length through the pitagorean theorem '''
    diff_x = start[0] - end[0]
    diff_y = start[1] - end[1]

    return math.sqrt(diff_x**2 + diff_y**2)

class LineFeatures:
    def line_extraction_callback(self, msg):
        self.line_list = np.array(msg.line_segments)
        
    def __init__(self):
        self.line_list = []
        self.sub_lines = rospy.Subscriber('line_segments', LineSegmentList, self.line_extraction_callback)

    def print_line(self, line):
        print 'New line'
        print '\trho = ', line.radius
        print '\tangle = ', line.angle
        print '\tstart = ', line.start[0], line.start[1]
        print '\tend = ', line.end[0], line.end[1]
        print '\tlength = ', line_length(line.start, line.end)

    def print_lines(self):
        print 'Lines avaliable:'
        if self.line_list is not None:
            map(self.print_line, self.line_list)
        print 'Next iteration'

    def run(self):
        self.print_lines()
