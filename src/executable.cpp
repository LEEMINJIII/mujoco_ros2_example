/**
 * @file   executable.cpp
 * @author Jon Woolfrey
 * @email  jonathan.woolfrey@gmail.com
 * @date   February 2025
 * @version 1.0
 * @brief  Starts ROS2 and runs the MuJoCoNode.
 * 
 * @details This contains the main() function for the C++ executable.
 *          Its purpose is to start ROS2, load parameters, then create & run an instance of the
 *          MuJoCoNode class.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://mujoco.org/ for more information about MuJoCo
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation
 */
#include <MuJoCoNode.h>
#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    
    auto node = std::make_shared<rclcpp::Node>("mujoco_sim_parameters");                            // Create a node to access parameters

    // Simulation parameters
    int simulationFrequency    = node->declare_parameter<int>("simulation_frequency", 500);
    int visualizationFrequency = node->declare_parameter<int>("visualization_frequency", 20);
    std::string xmlLocation    = node->declare_parameter<std::string>("xml", "");
    std::string controlMode    = node->declare_parameter<std::string>("mode", "TORQUE");
    std::string publisherName  = node->declare_parameter<std::string>("publisher_name", "joint_states");
    std::string subscriberName = node->declare_parameter<std::string>("subscriber_name", "joint_commands");

    // Load camera parameters
    std::vector<double> camera_focal_point = node->declare_parameter<std::vector<double>>("camera_focal_point", {0.0, 0.0, 0.0});
    double camera_distance = node->declare_parameter<double>("camera_distance", 1.0);
    double camera_azimuth = node->declare_parameter<double>("camera_azimuth", 0.0);
    double camera_elevation = node->declare_parameter<double>("camera_elevation", 0.0);
    bool camera_orthographic = node->declare_parameter<bool>("camera_orthographic", false);
        
    // Set the control mode
    ControlMode control_mode;
         if (controlMode == "POSITION") control_mode = POSITION;
    else if (controlMode == "VELOCITY") control_mode = VELOCITY;
    else if (controlMode == "TORQUE")   control_mode = TORQUE;
    else                                control_mode = UNKNOWN;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    
    try
    {
        auto mujocoSim = std::make_shared<MuJoCoNode>(xmlLocation,
                                                      publisherName,
                                                      subscriberName,
                                                      control_mode,
                                                      simulationFrequency,
                                                      visualizationFrequency);

        mujocoSim->set_camera_properties({camera_focal_point[0], camera_focal_point[1], camera_focal_point[2]},
                                          camera_distance,
                                          camera_azimuth, 
                                          camera_elevation,
                                          camera_orthographic);                             

        rclcpp::spin(mujocoSim);                                                                    // Run the simulation indefinitely
    }
    catch(const std::exception &exception)
    {
        std::cerr << exception.what() << "\n";
    }
    
    rclcpp::shutdown();                                                                             // Shut down
    
    return 0;
}

