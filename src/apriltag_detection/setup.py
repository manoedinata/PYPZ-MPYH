from setuptools import find_packages, setup

package_name = 'apriltag_detection'

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
    maintainer='iris',
    maintainer_email='iris@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_detection = apriltag_detection.apriltag_detection:main',
            'yolo_detection = apriltag_detection.yolo_detection:main',
            'onnx_lane_detection = apriltag_detection.onnx_lane_detection:main',
            'onnx_sign_detection = apriltag_detection.onnx_sign_detection:main',
        ],
    },
)
