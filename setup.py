#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_plotbag', 'rqt_plotbag.data_plot'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_plotbag']
)

setup(**d)
