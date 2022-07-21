from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['pytorch_sac'],
    scripts=['scripts/uav_collision_avoidance_node.py'],
    package_dir={'': 'scripts'}
)

setup(**d)