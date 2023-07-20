 ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['get_model'],
    package_dir={'': 'pointnet2_cls_msg'},
)
setup_args = generate_distutils_setup(
    packages=['PointViewGCN'],
    package_dir={'': 'PintView_GCN'},
)


setup(**setup_args)