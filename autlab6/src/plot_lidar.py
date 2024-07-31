#!/usr/bin/env python3

import numpy as np
from matplotlib import pyplot as plt
import warnings
warnings.filterwarnings("ignore",".*GUI is implemented.*")

import rospy
from autlab2.msg import ArrayXY


def plot_xy(msg):
  plt.plot(msg.x, msg.y, 'b.')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.grid()
  plt.axis("equal")
  plt.draw()
  plt.pause(0.001)
  plt.clf()

   
if __name__ == '__main__':
  rospy.init_node("plotter_xy")
  rospy.Subscriber("lidar_xy", ArrayXY, plot_xy)
  plt.show()
  rospy.spin()

