from setuptools import setup

setup(
    name='tesla',
    version='0.1.0',
    packages=['tesla'],
    scripts=['scripts/obstacle_watch.py'], 
    install_requires=[
        'numpy', 
        'rospy',
    ],
    author='Corey Knight',
    keywords='ROS',
)