#!/usr/bin/env python
"""Setup config file."""

from os import path

from setuptools import find_packages, setup


here = path.abspath(path.dirname(__file__))

with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()


setup(
    name='reachy-sdk',
    version='0.2.1',
    packages=find_packages(exclude=['tests']),

    install_requires=[
        'numpy',
        'opencv-python',
        'reachy-sdk-api',
        'grpcio',
        'protobuf>3',
    ],

    extras_require={
        'doc': ['sphinx'],
    },

    author='Pollen Robotics',
    author_email='contact@pollen-robotics.com',
    url='https://github.com/pollen-robotics/reachy-sdk',

    description='Python Reachy SDK.',
    long_description=long_description,
    long_description_content_type='text/markdown',
)
