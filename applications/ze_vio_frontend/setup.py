#!/usr/bin/env python
'''
This code is provided for internal research and development purposes by Huawei solely,
in accordance with the terms and conditions of the research collaboration agreement of May 7, 2020.
Any further use for commercial purposes is subject to a written agreement.
'''

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ze_vio_frontend'],
    package_dir={'': 'py'},
    install_requires=['yaml'],
    )

setup(**d)
