from setuptools import setup
package_name = 'drawbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/drawbot/launch/', ['share/drawbot/launch/drawbot.py']),
        ('share/drawbot/description/', ['share/drawbot/description/drawbot.urdf'])
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
            'motors = drawbot.motors:main',
            'uwb = drawbot.uwb:main',
        ],
    },
)
