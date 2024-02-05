from setuptools import setup, find_packages

setup(
name='Namak',
version='1.0',
author='Hamza Javed',
author_email='javed.hamza80@gmail.com',
packages=find_packages(),
license='LICENSE.txt',
description='Functions to make my life easier',
install_requires=[
    "PyYAML",
    "numpy",
    "opencv-python",
],
)
