#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  9 15:39:34 2016

@author: kaihong
"""
from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize

import sys
import numpy

ext_modules = [
    Extension("orb_slam",
              sources = ["orb_slam.pyx"],
              language='c++',
              include_dirs = ["../../include/",
                              "../../",
                              "/usr/include/eigen3",
                              numpy.get_include(),],
              library_dirs = [ "/opt/ros/indigo/lib/"],
              libraries = ["boost_thread"],
              extra_objects = ["../../lib/libORB_SLAM2.so"],
              extra_compile_args = ["-std=c++11"]
)]

setup(
  name = 'orb_slam for python',
  ext_modules = cythonize(ext_modules),
)
