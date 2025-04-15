from setuptools import setup

package_name = 'coppelia_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='il_tuo_nome',
    maintainer_email='tu@email.com',
    description='Descrizione del pacchetto',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # mapping tra nome CLI ed eseguibile Python
            'obj_dect_with_tf = coppelia_pose_estimate.obj_dect_with_tf:main'
        ],
    },
)
