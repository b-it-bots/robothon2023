
#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['robothon2023'],
   package_dir={'robothon2023': 'src/robothon2023'}
)

setup(**d)
