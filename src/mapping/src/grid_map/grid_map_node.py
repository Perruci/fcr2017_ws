#!/usr/bin/env python
import rospy
import grid_map

if __name__ == '__main__':

    rospy.init_node('grid_map')

    try:
        grid = grid_map.GridMap()
        grid.run()
    except rospy.ROSInterruptException:
        pass

    # r = rospy.Rate(1) # 1hz
    #
    # while not rospy.is_shutdown():
    #     try:
    #         top_map.run()
    #         r.sleep()
    #     except rospy.ROSInterruptException:
    #         pass
