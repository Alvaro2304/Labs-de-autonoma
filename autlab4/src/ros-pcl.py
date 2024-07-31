#!/usr/bin/env python3

# Import modules
import rospy
import pcl
import numpy as np
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2
from pcl_helper import *

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from random import randint

class puntos(object):
    def __init__(self,topic_name='/camera/depth_registered/points'):
        self.puntos=PointCloud2()
        isub=rospy.Subscriber(topic_name,PointCloud2,self.data_callback)
        self.flag = False

    def data_callback(self,msg):
        self.data=msg
        self.puntos=ros2pcl(self.data)
        self.flag = True
    def get_data(self):
       return self.puntos

if __name__ == '__main__':
  # Inicializar el nodo de ROS
  rospy.init_node('puntos_node')
  rospcl=puntos()

  topic_pub='topico_mesa'
  topic_pub2='topico_objetos'
  pubdata=rospy.Publisher(topic_pub,PointCloud2,queue_size=10)
  pubdata2=rospy.Publisher(topic_pub2,PointCloud2,queue_size=10)

  # Tiempo de ejecución del bucle (en Hz)
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    # Obtener data
    if(rospcl.flag):
      cloud=rospcl.get_data()
      passthrough = cloud.make_passthrough_filter()
      # Assignar el eje y el rango para el filtro.
      filter_axis = 'z'
      passthrough.set_filter_field_name(filter_axis)
      min_val = 1
      max_val = 5

      #min_val = 1.0
      #max_val = 1.5
      passthrough.set_filter_limits(min_val, max_val)

      # Usar el filtro para obtener la nube de puntos resultante
      cloud_filtered = passthrough.filter()
      cloud4ros=pcl2ros(cloud_filtered)
      pubdata.publish(cloud4ros)

      seg=cloud_filtered.make_segmenter()
      seg.set_model_type(pcl.SACMODEL_PLANE)
      seg.set_method_type(pcl.SAC_RANSAC)

      # Máxima distancia
      max_distance = 0.015
      # max_distance = 1
      seg.set_distance_threshold(max_distance)
      # Función de segmentación con RANSAC para obtener los índices de los inliers
      inliers, coefficients = seg.segment()
        
      # Extracción de  inliers
      cloud_inliers = cloud_filtered.extract(inliers, negative=True)

      cloud4ros2=pcl2ros(cloud_inliers)

      pubdata2.publish(cloud4ros2)
      # Esperar
      rate.sleep()