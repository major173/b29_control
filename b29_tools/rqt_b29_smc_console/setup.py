# # ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['rqt_b29_smc_console'],
    package_dir={'': 'src'},
    requires=['rospy', 'rqt_gui_py', 'python_qt_binding'],
)

setup(**setup_args)
