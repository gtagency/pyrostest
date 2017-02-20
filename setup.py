#! /usr/bin/env python
# -*- coding: utf-8 -*-

import codecs
import os

from setuptools import setup

here = os.path.abspath(os.path.dirname(__file__))

with codecs.open(os.path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = '\n' + f.read()

required = [
    'psutil==5.1.3'
    ]

test = []

setup(
    name='pyrostest',
    version='0.0.4',
    description='The most lit ros testing framework',
    long_description=long_description,
    packages=['pyrostest'],
    install_requires=required,
    url='https://github.com/gtagency/pyrostest',
    author='Joshua Morton',
    author_email='joshua.morton13@gmail.com',
    maintainer='Raphael Gontijo Lopes',
    maintainer_email='raphaelgontijolopes@gmail.com',
    license='MIT',
    classifiers=[
        'Programming Language :: Python',
        'Programming Language :: Python :: 2.7',
        ],
    )


