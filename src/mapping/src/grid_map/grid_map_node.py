#!/usr/bin/env python
import rospy
import cv2
import grid_map

if __name__ == '__main__':

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
            pass
