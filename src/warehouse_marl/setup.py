import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'warehouse_marl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'dashboard'),
            glob(os.path.join('dashboard', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARTI Master Student',
    maintainer_email='ines.abdellaziz@enit.utm.tn',
    description=(
        'Coordination distribuee multi-agents (graphe dynamique, RFID, '
        'consensus, allocation de taches par DRL) pour entrepot Industry 4.0.'
    ),
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # Couche capteurs / graphe
            'rfid_sensor_node = warehouse_marl.rfid_sensor_node:main',
            'graph_publisher_node = warehouse_marl.graph_publisher_node:main',
            # Couche coordination distribuee (consensus)
            'consensus_node = warehouse_marl.consensus_node:main',
            # Couche allocation de taches par DRL
            'drl_allocator_node = warehouse_marl.drl_allocator_node:main',
            'task_generator_node = warehouse_marl.task_generator_node:main',
            'mission_executor_node = warehouse_marl.mission_executor_node:main',
            'lifecycle_watchdog_node = warehouse_marl.lifecycle_watchdog_node:main',
            # Couche RFID : inventaire probabiliste + evaluation
            'inventory_node = warehouse_marl.inventory_node:main',
            'rfid_metrics_logger = warehouse_marl.rfid_metrics_logger:main',
            'rfid_offline_experiment = warehouse_marl.rfid_experiment:offline_main',
            'plot_rfid_metrics = warehouse_marl.rfid_experiment:plot_main',
            # Tableau de bord web (carte, RFID, missions)
            'warehouse_dashboard = warehouse_marl.dashboard_server:main',
            # Entrainement hors-ligne du modele DRL (pas un noeud ROS2)
            'train_drl = warehouse_marl.drl.train_drl:main',
        ],
    },
)
