#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_peopleflow', 'rqt_peopleflow.plugins'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_peopleflow.py']
)

setup(**d)