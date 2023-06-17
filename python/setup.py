#!/usr/bin/env python3

'''
Python distutils setup file for multirotor dynamics module

Copyright (C) 2023 Simon D. Levy

MIT License
'''

from setuptools import setup

setup(
    name='multirotor_dynamics',
    version='0.1',
    install_requires=['numpy'],
    description='Gym environment for multicopters',
    packages=['multirotor_dynamics'],
    author='Simon D. Levy',
    author_email='simon.d.levy@gmail.com',
    url='https://github.com/simondlevy/MultirotorDynamics',
    license='MIT',
    platforms='Linux; Windows; OS X'
    )
