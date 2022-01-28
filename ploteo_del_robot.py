#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep  4 22:01:50 2021

@author: federico
"""

import matplotlib.pyplot as plt
import numpy as np

#==================================================================
# función que permite plotear el robot
def plotear_robot(axes,x,y,theta,radio_robot,col):
	circle = plt.Circle((x,y),radius = radio_robot,fill = False,color = col)
	axes.add_artist(circle)
	bearing = plt.Line2D([x,x + radio_robot * np.cos(theta)],[y,y + radio_robot * np.sin(theta)],linewidth=2, color = col)
	axes.add_line(bearing)

	return (circle,bearing)

#==================================================================
# función que actualiza el ploteo del robot
def actualizar_ploteo_robot(dibujo_robot,x,y,theta,radio_robot):
	dibujo_robot[0].center = (x,y)
	dibujo_robot[1].set_xdata([x,x+radio_robot * np.cos(theta)])
	dibujo_robot[1].set_ydata([y,y+radio_robot * np.sin(theta)])