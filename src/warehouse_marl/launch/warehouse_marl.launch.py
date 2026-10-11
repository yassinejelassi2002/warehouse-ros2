"""
warehouse_marl.launch.py
--------------------------
Lance toute la couche "coordination distribuee + RFID + DRL" par-dessus
la simulation Nav2/Gazebo deja fournie par rosnav.

Usage typique (2 terminaux) :

    # Terminal 1 - simulation + navigation (package rosnav) :
    ros2 launch rosnav_bot multi_robot.launch.py \\
        world:=warehouse robot_count:=4 explore:=true fleet_mgmt:=false

    # Terminal 2 - couche coordination multi-agents (ce package) :
    ros2 launch warehouse_marl warehouse_marl.launch.py \\
        robot_names:="['robot1','robot2','robot3','robot4']" drl_mode:=heuristic \\
        map_origin_x:=-1.5 map_origin_y:=-1.0

Ce launch file demarre, pour N robots :
    - 1x graph_publisher_node       (etat partage du graphe dynamique)
    - 1x rfid_sensor_node           (couche capteurs RFID/IoT simulee)
    - 1x task_generator_node        (flux de commandes a traiter)
    - 1x inventory_node             (inventaire RFID probabiliste, scans)
    - 1x mission_executor_node      (assignations -> actions Nav2)
    - Nx consensus_node             (heuristic/hybrid seulement)
    - 1x drl_allocator_node         (hybrid/direct seulement)
"""

import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_names_str = LaunchConfiguration('robot_names').perform(context)
    # Autorise soit une liste Python ("['robot1','robot2']"), soit une
    # chaine separee par des virgules ("robot1,robot2").
    if robot_names_str.strip().startswith('['):
        robot_names = ast.literal_eval(robot_names_str)
    else:
        robot_names = [r.strip() for r in robot_names_str.split(',') if r.strip()]

    drl_mode = LaunchConfiguration('drl_mode').perform(context)
    model_path = LaunchConfiguration('model_path').perform(context)
    map_origin_x = float(LaunchConfiguration('map_origin_x').perform(context))
    map_origin_y = float(LaunchConfiguration('map_origin_y').perform(context))
    map_topic = LaunchConfiguration('map_topic').perform(context)
    use_rfid_inventory = LaunchConfiguration('use_rfid_inventory').perform(context).lower() == 'true'
    rfid_drift_period = float(LaunchConfiguration('rfid_drift_period').perform(context))
    demand_skew = float(LaunchConfiguration('demand_skew').perform(context))
    if drl_mode not in ('heuristic', 'hybrid', 'direct'):
        raise ValueError('drl_mode must be heuristic, hybrid, or direct')

    common = {
        'use_sim_time': True,
        'map_origin_x': map_origin_x,
        'map_origin_y': map_origin_y,
    }

    nodes = []

    nodes.append(Node(
        package='warehouse_marl',
        executable='graph_publisher_node',
        name='graph_publisher_node',
        output='screen',
        parameters=[{**common, 'robot_names': robot_names}],
    ))

    nodes.append(Node(
        package='warehouse_marl',
        executable='rfid_sensor_node',
        name='rfid_sensor_node',
        output='screen',
        parameters=[{**common, 'robot_names': robot_names,
                     'drift_period_sec': rfid_drift_period}],
    ))

    nodes.append(Node(
        package='warehouse_marl',
        executable='inventory_node',
        name='inventory_node',
        output='screen',
        parameters=[common],
    ))

    nodes.append(Node(
        package='warehouse_marl',
        executable='task_generator_node',
        name='task_generator_node',
        output='screen',
        parameters=[{**common, 'map_topic': map_topic,
                     'use_rfid_inventory': use_rfid_inventory,
                     'demand_skew': demand_skew}],
    ))

    if drl_mode in ('hybrid', 'direct'):
        nodes.append(Node(
            package='warehouse_marl',
            executable='drl_allocator_node',
            name='drl_allocator_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_names': robot_names,
                'mode': drl_mode,
                'model_path': model_path,
            }],
        ))

    if drl_mode != 'direct':
        for name in robot_names:
            nodes.append(Node(
                package='warehouse_marl',
                executable='consensus_node',
                name=f'consensus_node_{name}',
                namespace=name,
                output='screen',
                parameters=[{**common, 'robot_id': name}],
            ))

    nodes.append(Node(
        package='warehouse_marl',
        executable='mission_executor_node',
        name='mission_executor_node',
        output='screen',
        parameters=[{**common, 'robot_names': robot_names}],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_names', default_value="['robot1','robot2','robot3','robot4']",
            description="Liste des noms de robots, ex: \"['robot1','robot2']\""
        ),
        DeclareLaunchArgument(
            'drl_mode', default_value='heuristic',
            description="Allocation: 'heuristic', 'hybrid', ou 'direct'"
        ),
        DeclareLaunchArgument(
            'model_path', default_value='warehouse_dqn.pt',
            description="Chemin vers le modele DQN entraine (.pt)"
        ),
        DeclareLaunchArgument(
            'map_origin_x', default_value='-1.5',
            description="Coordonnee Gazebo x de l origine map (spawn robot1)"
        ),
        DeclareLaunchArgument(
            'map_origin_y', default_value='-1.0',
            description="Coordonnee Gazebo y de l origine map (spawn robot1)"
        ),
        DeclareLaunchArgument(
            'use_rfid_inventory', default_value='true',
            description="Picking selon la croyance RFID (true) ou le WMS seul (false)"
        ),
        DeclareLaunchArgument(
            'rfid_drift_period', default_value='20.0',
            description="Secondes entre deux evenements d'inventaire non enregistres (0 = aucun)"
        ),
        DeclareLaunchArgument(
            'demand_skew', default_value='0.0',
            description="Exposant Zipf de la demande par rack (0 = uniforme)"
        ),
        DeclareLaunchArgument(
            'map_topic', default_value='/map_fused',
            description="Carte fusionnee pour les cibles accessibles; /map sert de repli"
        ),
        OpaqueFunction(function=launch_setup),
    ])
