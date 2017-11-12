#!/usr/bin/env python
import rospy
import line_features

if __name__ == '__main__':

    rospy.init_node('localization_line_features')

    line_feat = line_features.LineFeatures()
    r = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        try:
            line_feat.run()
            r.sleep()
        except rospy.ROSInterruptException:
            pass
