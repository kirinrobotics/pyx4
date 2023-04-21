from setuptools import setup

package_name = 'pyx4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rtr',
    maintainer_email='haiquantran2897@gmail.com',
    description='pyx4',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pyx4_gps = pyx4.pyx4_gps:main',
            'pyx4_offboard_control = pyx4.offboard_control:main',
            'pyx4_multiros = pyx4.pyx4_multiros:main',
        ],
    },
)
