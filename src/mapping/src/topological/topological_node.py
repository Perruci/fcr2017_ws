#!/usr/bin/env python
import rospy
import topological_map

if __name__ == '__main__':

    rospy.init_node('topological')

    top_map = topological_map.TopologicalMap()
    r = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        try:
            top_map.run()
            r.sleep()
        except rospy.ROSInterruptException:
            pass
