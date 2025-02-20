import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    mujoco_interface_dir = get_package_share_directory('mujoco_ros2')                               # Get the path to mujoco_node package

    # Load config files
    config_dir = os.path.join(mujoco_interface_dir, 'config')
    camera_params = os.path.join(config_dir, 'default_camera.yaml')
    sim_params = os.path.join(config_dir, 'default_sim.yaml')

    # Define the LaunchConfiguration for xml_path using the path inside mujoco_interface
    xml = LaunchConfiguration('xml', default=os.path.join(mujoco_interface_dir, 'model/kuka_iiwa_14/scene.xml'))

    return LaunchDescription([
        Node
        (
            package    = 'mujoco_ros2',
            executable = 'mujoco_node',
            output     = 'screen',
            parameters =
            [
                {'mode': 'VELOCITY'},
                camera_params,
                sim_params,
                {'xml': xml}
            ]
        )
    ])
