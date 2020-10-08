import rospy
import random as rand
from geometry_msgs.msg import PointStamped, Point

def main():
    pub = rospy.Publisher("biotac/0/contact_point", PointStamped, queue_size=1)
    rospy.init_node("biotac_publisher")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pstamped = PointStamped()
        pstamped.header.stamp = rospy.Time.now()
        point = Point()
        point.x = rand.random() * 0.01
        point.y = rand.random() * 0.01
        point.z = rand.random() * 0.01
        pstamped.point = point
        raw_input("press enter to publish")
        pub.publish(pstamped)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
