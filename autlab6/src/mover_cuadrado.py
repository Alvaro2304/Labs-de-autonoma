#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('cuadrado_node')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist=Twist()
    lado_cuadrado=1.0
    flag=True
    complete=True
    # Frecuencia del bucle principal
    freq = 10
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        if(flag):
            for i in range(5):
                twist.linear.x=0.2
                twist.angular.z=0.0
                pub.publish(twist)
                rospy.sleep(lado_cuadrado/twist.linear.x)
                twist.linear.x=0.0
                twist.angular.z=0.4
                pub.publish(twist)
                rospy.sleep(4)
            flag=False
        if(complete):
            rospy.loginfo("Cuadrado completado")
            complete=False
        twist.linear.x=0.0
        twist.angular.z=0.0
        pub.publish(twist)

        rate.sleep()