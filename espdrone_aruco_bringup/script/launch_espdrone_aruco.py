import argparse
import os
import yaml
from typing import List, Dict
import roslaunch
import rospkg

BRINGUP_PACKAGE_NAME = "espdrone_aruco_bringup"


def read_yaml_config(file_path: str) -> Dict[str, str]:
    with open(file_path, 'r') as config:
        return yaml.safe_load(config)


def get_server_launch_config(env_config: Dict[str, str]):
    server_launch = [BRINGUP_PACKAGE_NAME, "espdrone_server_and_map.launch"]

    server_launch.extend([f"marker_map_frame:={env_config['marker_map_frame']}",
                          f"world_fixed_frame:={env_config['world_fixed_frame']}",
                          f"world_to_marker_map_tf:={env_config['world_to_marker_map_tf']}"
                         ])
    
    server_launch_file = roslaunch.rlutil.resolve_launch_arguments(server_launch)[0]
    server_launch_args = server_launch[2:]
    launch_list = [(server_launch_file, server_launch_args)]

    return launch_list


def get_espdrone_launch_config(drone_names: List[str], env_config: Dict[str, str]):
    launch_list = []
    espdrone_launch_file = [BRINGUP_PACKAGE_NAME, "espdrone_aruco.launch"]
    bringup_package_path = rospkg.RosPack().get_path(BRINGUP_PACKAGE_NAME)

    for drone_name in drone_names:
        drone_config_path = os.path.join(bringup_package_path, "config", "espdrone", f"{drone_name}.yaml")
        drone_config = read_yaml_config(drone_config_path)
        
        camera_info_full_path = os.path.join(bringup_package_path, "config", "espdrone", "camera_calib", drone_config['camera_info_file'])
        aruco_map_full_path = os.path.join(bringup_package_path, "config", "environment", "aruco_maps", env_config['aruco_map_config_file'])
        drone_launch = espdrone_launch_file.copy()
        drone_launch.extend([f"drone_name:={drone_name}",
                             f"drone_ip_addr:={drone_config['drone_ip_addr']}",
                             f"camera_info_file:={camera_info_full_path}",
                             f"visualize_output:={drone_config['visualize_output']}",
                             f"aruco_marker_size:={env_config['aruco_marker_size']}",
                             f"aruco_map_config_file:={aruco_map_full_path}",
                             f"marker_map_frame:={env_config['marker_map_frame']}",
                             f"world_fixed_frame:={env_config['world_fixed_frame']}"
                            ])
        drone_launch_file = roslaunch.rlutil.resolve_launch_arguments(drone_launch)[0]
        drone_launch_args = drone_launch[2:]
        launch_list.append((drone_launch_file, drone_launch_args))

    return launch_list


def main():
    parser = argparse.ArgumentParser(description="Launch multiple ESP-drones inside one environment/map with ArUco markers")
    parser.add_argument("--drones", '-d', type=str, nargs='+', required=True, metavar="<drone_names>", help="Drone(s) to launch, exclude '.yaml'")
    parser.add_argument("--env", '-e', type=str, required=True, metavar="<env_name>",
                         help="Environment (map) with ArUco markers to fly in, exclude '.yaml'")
    args = parser.parse_args()

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    # Get configurations of the map and ArUco markers in it.
    bringup_package_path = rospkg.RosPack().get_path(BRINGUP_PACKAGE_NAME)
    env_config_path = os.path.join(bringup_package_path, "config", "environment", f"{args.env}.yaml")
    env_config = read_yaml_config(env_config_path)

    # Launch ESP-drone server and other required nodes (one node for all drones).
    launch_list = get_server_launch_config(env_config)

    # Launch each ESP-drone with their own configuration (IP address, etc.).
    launch_list.extend(get_espdrone_launch_config(args.drones, env_config))

    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_list)
    parent.start()

    # # Launch individual nodes.
    # static_tf_node = roslaunch.core.Node(package="tf", node_type="static_transform_publisher", name="world_to_marker_static_tf",
    #                                      args=f"{env_config['world_to_marker_map_tf']} {env_config['world_fixed_frame']} \
    #                                             {env_config['marker_map_frame']} 50"
    #                                     )
    # parent.runner.launch_node(static_tf_node)

    try:
        parent.spin()

    finally:
        parent.shutdown()


if __name__ == "__main__":
    main()