from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['admittance_controller_smach'],
    package_dir={'': 'src'}
)

setup(**d)
