"""
warehouse_full.launch.py
--------------------------
Lance tout le systeme en une seule commande :

    - Gazebo (monde warehouse) + RViz
    - N robots : SLAM (robot1) / AMCL (autres), Nav2, fusion des scans
    - exploration de frontieres (explore:=true)
    - couche coordination multi-agents warehouse_marl (graphe, RFID,
      generateur de taches, consensus / DRL, mission executor)
    - lifecycle_watchdog (termine les demarrages AMCL/Nav2 bloques)
    - tableau de bord web : http://localhost:8080 (dashboard:=false pour le
      desactiver)

Usage :

    ros2 launch warehouse_marl warehouse_full.launch.py
    ros2 launch warehouse_marl warehouse_full.launch.py robot_count:=4
    ros2 launch warehouse_marl warehouse_full.launch.py explore:=false
    ros2 launch warehouse_marl warehouse_full.launch.py \\
        drl_mode:=hybrid model_path:=$PWD/warehouse_dqn_3robots.pt

Les noms des robots (robot1..robotN) sont generes a partir de robot_count et
transmis a la couche MARL. La couche MARL demarre apres la fusion de cartes
(meme formule de delai que multi_robot.launch.py).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Valeurs par defaut de multi_robot.launch.py (rosnav_bot) utilisees pour
# calculer le moment ou la fusion de cartes demarre.
_AMCL_START_DELAY = 13.0
_ROBOT_START_STAGGER = 6.0


def _truthy(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    robot_count = int(LaunchConfiguration('robot_count').perform(context))
    explore = LaunchConfiguration('explore').perform(context)
    slam_mode = LaunchConfiguration('slam_mode').perform(context).strip().lower()
    merge_scans = LaunchConfiguration('merge_scans').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    marl = _truthy(LaunchConfiguration('marl').perform(context))
    drl_mode = LaunchConfiguration('drl_mode').perform(context)
    model_path = LaunchConfiguration('model_path').perform(context)
    map_origin_x = LaunchConfiguration('map_origin_x').perform(context)
    map_origin_y = LaunchConfiguration('map_origin_y').perform(context)

    if robot_count < 1:
        raise ValueError('robot_count must be >= 1')
    robot_names = [f'robot{i + 1}' for i in range(robot_count)]

    rosnav_launch = os.path.join(
        get_package_share_directory('rosnav_bot'), 'launch', 'multi_robot.launch.py')
    marl_launch = os.path.join(
        get_package_share_directory('warehouse_marl'), 'launch', 'warehouse_marl.launch.py')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rosnav_launch),
        launch_arguments={
            'world': 'warehouse',
            'robot_count': str(robot_count),
            'explore': explore,
            'slam_mode': slam_mode,
            'merge_scans': merge_scans,
            'fleet_mgmt': 'false',
            'headless': headless,
            'rviz': rviz,
        }.items())

    actions = [
        LogInfo(msg=f'[warehouse_full] robots = {robot_names}'),
        simulation,
    ]
    if _truthy(LaunchConfiguration('dashboard').perform(context)):
        actions.append(Node(
            package='warehouse_marl',
            executable='warehouse_dashboard',
            name='warehouse_dashboard',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_names': robot_names,
                'port': int(LaunchConfiguration('dashboard_port').perform(context)),
                'map_origin_x': float(LaunchConfiguration('map_origin_x').perform(context)),
                'map_origin_y': float(LaunchConfiguration('map_origin_y').perform(context)),
            }]))
    actions += [
        # Finishes robot AMCL/Nav2 bringups whose lifecycle reply was lost.
        Node(
            package='warehouse_marl',
            executable='lifecycle_watchdog_node',
            name='lifecycle_watchdog',
            output='screen',
            parameters=[{'robot_names': robot_names}]),
    ]

    if marl:
        if _truthy(explore) and slam_mode == 'multi':
            map_topic = '/map_merged'
        elif _truthy(explore) and _truthy(merge_scans) and robot_count > 1:
            map_topic = '/map_fused'
        else:
            map_topic = '/map'
        fusion_start = max(
            20.0, _AMCL_START_DELAY + (robot_count - 1) * _ROBOT_START_STAGGER + 12.0)
        marl_delay = fusion_start + 10.0

        coordination = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(marl_launch),
            launch_arguments={
                'robot_names': ','.join(robot_names),
                'drl_mode': drl_mode,
                'model_path': model_path,
                'map_origin_x': map_origin_x,
                'map_origin_y': map_origin_y,
                'map_topic': map_topic,
                'use_rfid_inventory': LaunchConfiguration('use_rfid_inventory').perform(context),
                'rfid_drift_period': LaunchConfiguration('rfid_drift_period').perform(context),
                'demand_skew': LaunchConfiguration('demand_skew').perform(context),
            }.items())

        actions += [
            LogInfo(msg=f'[warehouse_full] MARL ({drl_mode}, map={map_topic}) '
                        f'will start at t={marl_delay:.0f}s'),
            TimerAction(period=marl_delay, actions=[coordination]),
        ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_count', default_value='3',
            description='Nombre de robots (robot1..robotN)'),
        DeclareLaunchArgument(
            'explore', default_value='true',
            description='true: exploration de frontieres; false: robots attendent les missions'),
        DeclareLaunchArgument(
            'slam_mode', default_value='single',
            description="single: SLAM robot1 + AMCL autres; multi: SLAM par robot + fusion /map_merged"),
        DeclareLaunchArgument(
            'merge_scans', default_value='true',
            description='Fusionne les scans des robots AMCL dans /map_fused'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true: Gazebo sans interface graphique'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Lancer RViz'),
        DeclareLaunchArgument(
            'marl', default_value='true',
            description='Lancer la couche coordination warehouse_marl'),
        DeclareLaunchArgument(
            'drl_mode', default_value='heuristic',
            description="Allocation: 'heuristic', 'hybrid', ou 'direct'"),
        DeclareLaunchArgument(
            'model_path', default_value='warehouse_dqn.pt',
            description='Modele DQN (.pt), entraine avec --n-agents = robot_count'),
        DeclareLaunchArgument(
            'use_rfid_inventory', default_value='true',
            description='Picking selon la croyance RFID (true) ou le WMS seul (false)'),
        DeclareLaunchArgument(
            'rfid_drift_period', default_value='20.0',
            description="Secondes entre deux evenements d'inventaire non enregistres (0 = aucun)"),
        DeclareLaunchArgument(
            'demand_skew', default_value='0.0',
            description='Exposant Zipf de la demande par rack (0 = uniforme)'),
        DeclareLaunchArgument(
            'dashboard', default_value='true',
            description='Tableau de bord web (http://localhost:<dashboard_port>)'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        DeclareLaunchArgument(
            'map_origin_x', default_value='-1.5',
            description='x Gazebo du spawn de robot1 (origine de la carte)'),
        DeclareLaunchArgument(
            'map_origin_y', default_value='-1.0',
            description='y Gazebo du spawn de robot1 (origine de la carte)'),
        OpaqueFunction(function=launch_setup),
    ])
