from setuptools import setup

setup(
    name='tesla',
    version='0.1.0',
    packages=['tesla'],
    scripts=['scripts/obstacle_watch.py',
             'camtest.py',
             'tests/test.py'], 
    install_requires=[
        'numpy', 
        'rospy',
        'scipy',
    ],
    author='Corey Knight',
    keywords='ROS',
)