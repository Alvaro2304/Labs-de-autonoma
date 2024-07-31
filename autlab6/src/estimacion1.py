#!/usr/bin/env python3
# -*- coding: utf-8 -*-                                                                                             
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import tf

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

    Velocidad_x=0.0
    Velocidad_y=0.0

    posicion_x=0.0
    posicion_y=0.0

    orientacion_z=0.0

    dt=0.01

    A = np.array([[1,dt,dt*dt/2,0,0,0,0,0],
              [0,1,dt,0,0,0,0,0],
              [0,0,1,0,0,0,0,0,0],
              [0,0,0,1,dt,dt*dt,0,0],
              [0,0,0,0,1,dt,0,0],
              [0,0,0,0,0,1,0,0],
              [0,0,0,0,0,0,1,dt],
              [0,0,0,0,0,0,0,1]])

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        ax,ay,w=sub.get_value_robot()
        aceleracion_x=ax #esto importa
        aceleracion_y=ay
        velocidad_angular_z=w