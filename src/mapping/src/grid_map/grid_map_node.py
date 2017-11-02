#!/usr/bin/env python
import rospy
import cv2
import grid_map

import os, errno

def create_dir(directory):
    ''' Create a directory if its not found '''
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise

if __name__ == '__main__':
    ''' GridMap Node main function '''
    rospy.init_node('grid_map')

    grid = grid_map.GridMap()
    r = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        try:
            grid.run()
            grid.show_layer(grid.current_id, grid.current_id)
            cv2.waitKey(1)
            r.sleep()
        except rospy.ROSInterruptException:
            save_path = 'src/mapping/docs/grid_map_results/'
            create_dir(save_path)
            grid.save_all_layers(save_path)
            pass
