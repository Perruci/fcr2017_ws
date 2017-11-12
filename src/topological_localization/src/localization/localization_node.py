#!/usr/bin/env python
import rospy
import localization

def main():
    rospy.init_node('localization')

    localize = localization.Localization()
    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        try:
            localize.run()
            rate.sleep()
        except:
            pass

if __name__ == '__main__':
    main()
