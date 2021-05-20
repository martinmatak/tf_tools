import rospy
import random as rand
import tf
from geometry_msgs.msg import PointStamped, Point, Pose

def main():
    rospy.init_node("world_publisher")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        br.sendTransform((0.0744432974896,-1.01452819533 ,1.2077374635),
                         (-0.685286722254, 0.106017961757,
                          0.020948441311,-0.720210707289),
                          rospy.Time.now(),
                         "desired_pose", "world")

    while not rospy.is_shutdown():
        br.sendTransform((0, -0.8, 0.59),
                          tf.transformations.quaternion_from_euler(0, 0, 0),
                          rospy.Time.now(),
                         "estimated_object_pose", "world")
    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0),
                          (0, 0, 0, 1),
                          rospy.Time.now(),
                          "world", "box_link")

    #rospy.spin()
    '''
    pub_0 = rospy.Publisher("biotac/index_tip/contact_point", PointStamped, queue_size=1)
    pub_1 = rospy.Publisher("biotac/middle_tip/contact_point", PointStamped, queue_size=1)
    pub_2 = rospy.Publisher("biotac/ring_tip/contact_point", PointStamped, queue_size=1)
    pub_3 = rospy.Publisher("biotac/thumb_tip/contact_point", PointStamped, queue_size=1)
    
    pubs = [pub_0, pub_1, pub_2, pub_3]
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
        raw_input("press enter to publish a random contact")
        pubs[rand.randint(0,3)].publish(pstamped)
        rate.sleep()
    '''

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
