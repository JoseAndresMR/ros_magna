#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 21 2018

@author: josmilrom
"""
from sympy import Point3D, Line3D
p1, p2, p3 = Point3D(0, 0, 0), Point3D(1, 1, 1), Point3D(-1, 2, 0)
l1, l2 = Line3D(p1, p2), Line3D(p2, p3)
print(type(l1.angle_between(l2)))
print(l1.smallest_angle_between(l2))
