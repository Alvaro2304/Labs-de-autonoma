#!/usr/bin/env python3
# -*- coding: utf-8 -*-                                                                                             
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import tf
from matplotlib import pyplot as plt

class SuscriptorImu(object):
    def __init__(self):
        topic = '/imu'
        self.pub = rospy.Subscriber(topic, Imu, self.callback)
        self.linear_vel = [0.0, 0.0]
        self.imu = Imu()
        
    def callback(self, msg):
        self.imu = msg
    def get_imu(self):
        return self.imu
    def get_value_robot(self):
        """
        Retorna la aceleracion lineal (en el sistema local, del robot), y la velocidad angular

        """
        th = 2.0*np.arctan2(self.imu.orientation.z, self.imu.orientation.w)
        a = np.array([self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, 0.0])
        R = np.array([[np.cos(th), np.sin(th), 0.0], [-np.sin(th), np.cos(th), 0.0], [0.0, 0.0, 1.0]])
        af = np.dot(R, a)  
        return af[0], af[1], self.imu.angular_velocity.z   # ax, ay, w

if __name__ == "__main__":
    rospy.init_node('nodo_imu_integracion') # Inicializar el nodo
    sub = SuscriptorImu() # Crear el suscriptor
    # Inicializar broadcaster de TF
    br = tf.TransformBroadcaster()
    pub_pos = rospy.Publisher('posicion', Vector3, queue_size=10)
    pub_ori = rospy.Publisher('orientacion', Vector3, queue_size=10)
    

    Velocidad_x=0.0
    Velocidad_y=0.0
    Velocidad_z=0.0

    posicion_x=0.0
    posicion_y=0.0
    posicion_z=0.0

    orientacion_x=0.0
    orientacion_y=0.0
    orientacion_z=0.0

    dt=0.01

    x=0.0
    y=0.0
    theta=0
    posicion=Vector3()
    orientacion=Vector3()
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        
        ax,ay,w=sub.get_value_robot()

        aceleracion_x=ax #esto importa
        aceleracion_y=ay
        #aceleracion_z=sub.imu.linear_acceleration.z

        #velocidad_angular_x=sub.imu.angular_velocity.x
        #velocidad_angular_y=sub.imu.angular_velocity.y
        velocidad_angular_z=w
        
        Velocidad_x+=aceleracion_x*dt
        Velocidad_y+=aceleracion_y*dt
        #Velocidad_z+=+aceleracion_z*dt

        posicion_x+=Velocidad_x*dt
        posicion_y+=Velocidad_y*dt
        #posicion_z+=Velocidad_z*dt

        #orientacion_x+=velocidad_angular_x*dt
        #orientacion_y+=velocidad_angular_y*dt
        orientacion_z+=velocidad_angular_z*dt

        posicion.x=posicion_x
        posicion.y=posicion_y
        #posicion.z=posicion_z

        #orientacion.x=orientacion_x
        #orientacion.y=orientacion_y
        orientacion.z=orientacion_z

        pub_pos.publish(posicion)
        pub_ori.publish(orientacion)

        #x = np.cos(orientacion_z)*posicion_x
        #y = np.sin(orientacion_z)*posicion_x
        #theta = orientacion_z
        # Hacer broadcast del tf: 'base_footprint' con respecto a 'odom'
        br.sendTransform((posicion_x, posicion_y, 0.0),
                     tf.transformations.quaternion_from_euler(0.0, 0.0, orientacion_z),
                     rospy.Time.now(),
                     "base_footprint",
                     "odom")


        plt.plot(posicion_x, posicion_y, 'b.')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.grid()
        plt.axis("equal")
        plt.draw()
        plt.pause(0.001)
        plt.clf()
        plt.show()
        
        rate.sleep()
