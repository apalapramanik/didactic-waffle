 #!/usr/bin/env python
 ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['tb_apala'],
    package_dir={'': 'src'}
)

# # fetch values from package.xml
# setup_args = generate_distutils_setup(
#     packages=['get_model','PointNetSetAbstractionMsg'],
#     package_dir={'': 'pointnet2_cls_msg'},
# )



# setup_args = generate_distutils_setup(
#     packages=['marker'],
#     package_dir={'':'marker_publisher'},
# )

# setup_args = generate_distutils_setup(
#     packages=['FilterEstimator'],
#     package_dir={'':'kf_predictors'},
# )
# setup_args = generate_distutils_setup(
#     packages=['KalmanFilterEstimator'],
#     package_dir={'':'eval_pred'},
# )


# setup_args = generate_distutils_setup(
#     packages=['PointViewGCN'],
#     package_dir={'': 'PintView_GCN'},
# )


# setup(
#     version='...',
#     scripts=['src/pointnet2_cls_msg'],
#     packages=['tb_apala'],
#     package_dir={'': 'src'}
# )


setup(**setup_args)