#!/usr/bin/env python
import rospy
import grid_map

import os, errno

def create_dir(directory):
    ''' Create a directory if its not found '''
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
            print('Created directory: ', directory)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise

def main():
    ''' GridMap Node main function '''
    rospy.init_node('grid_map')

    grid = grid_map.GridMap()
    r = rospy.Rate(30) # 30 Hz
    while not rospy.is_shutdown():
        try:
            grid.run()
            grid.show_layer(grid.current_id, grid.current_id)
            r.sleep()
        except rospy.ROSInterruptException:
            save_path = 'src/mapping/docs/grid_map_results/'
            create_dir(save_path)
            grid.save_all_layers(save_path)
            pass

if __name__ == '__main__':
    main()
