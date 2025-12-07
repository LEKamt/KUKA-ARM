from setuptools import find_packages, setup
from glob import glob
package_name = 'fr_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luisk',
    maintainer_email='luisk@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rviz_bridge = fr_pkg.rviz_bridge:main',
            'control_servidor = fr_pkg.control_servidor:main',
            'user_cliente = fr_pkg.user_cliente:main',
            'enfoque_1 = fr_pkg.enfoque_1:main',
            'enfoque_2 = fr_pkg.enfoque_2:main',
            'fake_esp32 = fr_pkg.fake_esp32:main', # Simula ser un robot que por el momento no esta disponible
            'joint_cmd_test = fr_pkg.joint_cmd_test:main',
            'translator_node = fr_pkg.translator_node:main',
            'tray_cliente = fr_pkg.tray_cliente:main',
            'cosecha = fr_pkg.cosecha:main',
        ],
    },
)
