#!/usr/bin/env python
import rospy
import cic_probabilities

def main():
    rospy.init_node('localization')

    cic_prob = cic_probabilities.CIC_Probabilities()
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except:
            pass

if __name__ == '__main__':
    main()
