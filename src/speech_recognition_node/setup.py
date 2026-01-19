import os  # For constructing platform-independent file paths
from glob import glob  # To expand file patterns (e.g., launch/*.launch.py)
from setuptools import find_packages, setup  # setuptools functions for packaging

# Name of this ROS 2 Python package
package_name = 'speech_recognition_node'

setup(
    name=package_name,              # Package name (must match folder and XML)
    version='0.0.0',                # Initial version, update as you release
    packages=find_packages(exclude=['test']),  
    # Automatically find all Python packages in src/, excluding tests

    data_files=[
        # Tell ament where to index this package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install the package.xml into the share directory
        ('share/' + package_name, ['package.xml']),
        # Install launch files so `ros2 launch` can discover them
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'))
    ],

    install_requires=['setuptools'],  # Dependencies for pip install
    zip_safe=True,                    # Allow zipped installation (no unzipping needed)

    maintainer='pi',                  # Your name or username
    maintainer_email='pi@todo.todo',  # Your email (update before release)
    description='TODO: Package description',  # Short text describing the package
    license='TODO: License declaration',       # License type (e.g., Apache-2.0)

    tests_require=['pytest'],         # Testing framework requirement

    entry_points={
        # Create a console script so you can run:
        #   ros2 run speech_recognition_node speech_recognition_py
        'console_scripts': [
            'speech_recognition_py = speech_recognition_node.speech_recognition_py:main'
        ],
    },
)

'''

from setuptools import find_packages, setup

package_name = 'speech_recognition_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_recognition_py = speech_recognition_node.speech_recognition_py:main'
        ],
    },
)
'''
