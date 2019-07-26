#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
import rospy
import std_msgs.msg
import time
import math
import numpy as np
import tf, tf2_ros
import json
import copy
import random
import rospkg
import yaml
import io
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from xml.dom import minidom
from gazebo_msgs.srv import DeleteModel,SpawnModel
from visualization_msgs.msg import Marker
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, TorusArray, PolygonArray
from jsk_recognition_msgs.msg import Torus as jsk_Torus
from sympy import Point3D, Line3D, Segment3D
from sympy import Point as Point2D
from sympy import Polygon as Polygon2D
import xml.etree.ElementTree
# sys.path.insert(0, '/home/joseandresmr/catkin_ws/src/magna/Code/scripts')
# from Worlds import *
from GS import GroundStation