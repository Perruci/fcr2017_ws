import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

def talker():
    pub = rospy.Publisher('topological/where_to/pose', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        msg = Pose()
        msg.position = Point(38.5, 0, 0)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
