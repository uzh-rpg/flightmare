import os
import re
import sys
import platform
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion

setup(
    name='rpg_baselines',
    version='0.0.1',
    author='Yunlong Song',
    author_email='song@ifi.uzh.ch',
    description='Flightmare: A Quadrotor Simulator.',
    long_description='',
    install_requires=['gym==0.10.9', 'ruamel.yaml', 'numpy', 'stable_baselines==2.8'],
    packages=['rpg_baselines'],
)
