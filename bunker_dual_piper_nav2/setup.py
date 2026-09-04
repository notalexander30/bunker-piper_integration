import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bunker_dual_piper_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'CODEX_IMPLEMENTATION_PROMPT.md', 'CODEX_UPDATED_URDF_PROMPT.md', 'THIRD_PARTY_NOTICES.md', 'LICENSE', 'PIPER_LICENSE.txt']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        # Python may create scripts/__pycache__; install only real script files.
        ('share/' + package_name + '/scripts',
         [path for path in glob('scripts/*') if os.path.isfile(path)]),
        ('share/' + package_name + '/meshes/bunker', glob('meshes/bunker/*')),
        ('share/' + package_name + '/meshes/upper_frame', glob('meshes/upper_frame/*')),
        ('share/' + package_name + '/meshes/piper', glob('meshes/piper/*')),
        ('share/' + package_name + '/meshes/mounting', glob('meshes/mounting/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Bunker integration project',
    maintainer_email='student@example.invalid',
    description='Dual-PiPER Bunker Mini description and Nav2 integration.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joint_state_prefixer = bunker_dual_piper_nav2.joint_state_prefixer:main',
            'safe_cmd_vel_gate = bunker_dual_piper_nav2.safe_cmd_vel_gate:main',
        ],
    },
)
