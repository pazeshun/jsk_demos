#!/usr/bin/env python

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages, setup

d = generate_distutils_setup(
    packages=find_packages('python'),
    package_dir={'': 'python'},
)

setup(**d)
