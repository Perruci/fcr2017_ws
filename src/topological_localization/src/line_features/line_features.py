import rospy
import math
import numpy as np
from topological_localization.msg import LineSegmentList

def line_length(start, end):
    ''' Compute line length through the pitagorean theorem '''
    diff_x = start[0] - end[0]
    diff_y = start[1] - end[1]

    return math.sqrt(diff_x**2 + diff_y**2)

def wrap_angle(phases):
    ''' For a given angle, compute its equivalent in space -pi to pi  ref: https://stackoverflow.com/questions/15927755/opposite-of-numpy-unwrap'''
    wrap = ( phases + np.pi) % (2 * np.pi ) - np.pi
    return wrap



class LineFeatures:
    def line_extraction_callback(self, msg):
        self.line_list = np.array(msg.line_segments)

    def __init__(self):
        self.line_list = []
        self.sub_lines = rospy.Subscriber('line_segments', LineSegmentList, self.line_extraction_callback)
        self.parallel_min_dist = 1.5
        self.parallel_max_dist = 5
        self.continuity_tolerance = 1

    def  get_parallel_and_orthogonal(self, tolerance):
        ''' Extract line properties as parallel and orthogonal lines '''
        parallel_tuples = []
        orthogonal_tuples = []
        # create square matrix of size len(self.line_list)
        adjacency_matrix = np.empty([np.size(self.line_list), np.size(self.line_list)])
        for (i,j), value in np.ndenumerate(adjacency_matrix):
            diff_angle = wrap_angle(self.line_list[i].angle - self.line_list[j].angle)
            adjacency_matrix[i,j] = value
            # if i != j they are not the same line
            if i != j:
                if abs(diff_angle) < tolerance:
                    parallel_tuples.append((i,j))
                else:
                    orth_diff = abs(diff_angle - math.pi/2)
                    if orth_diff < tolerance:
                        orthogonal_tuples.append((i,j))

        # remove repeated tuples. ref: https://stackoverflow.com/questions/15352995/removing-permutations-from-a-list-of-tuples
        parallel_tuples = set(tuple(sorted(l)) for l in parallel_tuples)
        orthogonal_tuples = set(tuple(sorted(l)) for l in orthogonal_tuples)
        return parallel_tuples, orthogonal_tuples

    def parallel_validation(self, idx_tuple):
        ''' Returns true for parallel lines which distace is greater than min and lesser than max '''
        i, j = idx_tuple[0], idx_tuple[1]
        line1, line2 = self.line_list[i], self.line_list[j]

        dist = abs(line1.radius - line2.radius)
        if self.parallel_min_dist < dist and dist < self.parallel_max_dist:
            print 'parallel lines distance: ', dist
            return True
        else:
            return False

    def filter_parallels(self, parallel_pairs):
        ''' Filter parallel_pairs which distance is outside a range '''
        if parallel_pairs is None:
            return None
        # filter min_distance and max_distance
        parallel_pairs = filter(self.parallel_validation, parallel_pairs)
        return parallel_pairs

    def tuple_inside_tolerance(self, tup):
        if tup[0] < self.continuity_tolerance and tup[1] < self.continuity_tolerance:
            return True
        else:
            return False

    def orthogonal_validation(self, idx_tuple):
        ''' Check for continuity on the orthogonal pairs. Returns true for continuous, false otherwise. '''
        i, j = idx_tuple[0], idx_tuple[1]
        line1, line2 = self.line_list[i], self.line_list[j]

        # Subtract x and y coordinates of lines start and end
        diff_start = tuple(abs(np.subtract(line1.start, line2.start)))
        diff_end = tuple(abs(np.subtract(line1.end, line2.end)))
        diff_start_end = tuple(abs(np.subtract(line1.end, line2.end)))

        # detects low values in tuple pairs
        if self.tuple_inside_tolerance(diff_start) or self.tuple_inside_tolerance(diff_end) or self.tuple_inside_tolerance(diff_start_end):
            return True
        else:
            return False

    def filter_orthogonals(self, orthogonal_pairs):
        if orthogonal_pairs is None:
            return None
        # check for continuity between line start and end
        orthogonal_pairs = filter(self.orthogonal_validation, orthogonal_pairs)
        return orthogonal_pairs


    def process_line(self, tolerance=0.01):
        ''' Process resulting lines to classify their shape '''
        parallel_pairs, orthogonal_pairs = self.get_parallel_and_orthogonal(tolerance)

        parallel_pairs = self.filter_parallels(parallel_pairs)
        orthogonal_pairs = self.filter_orthogonals(orthogonal_pairs)

        print parallel_pairs
        print orthogonal_pairs

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
        self.process_line()
        self.print_lines()
