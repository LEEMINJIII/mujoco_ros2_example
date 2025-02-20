/**
 * @file   MuJoCoNode.h
 * @author Jon Woolfrey
 * @email  jonathan.woolfrey@gmail.com
 * @date   February 2025
 * @version 1.0
 * @brief  A class for connecting a MuJoCo simulation with ROS2 communication.
 * 
 * @details This class launches a MuJoCo simulation and provides communication channels in ROS2 for controlling it.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://mujoco.org/ for more information about MuJoCo
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation
 */

#ifndef MUJOCONODE_H
#define MUJOCONODE_H

#include <GLFW/glfw3.h>                                                                             // Graphics Library Framework; for visualisation
#include <iostream>                                                                                 // std::cerr, std::cout
#include <mujoco/mujoco.h>                                                                          // Dynamic simulation library
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries.
#include <sensor_msgs/msg/joint_state.hpp>                                                          // For publishing / subscribing to joint states.
#include <std_msgs/msg/float64_multi_array.hpp>

enum ControlMode {POSITION, VELOCITY, TORQUE, UNKNOWN};                                             // This needs a global scope
        
/**
 * @brief This class launches both a MuJoCo simulation, and ROS2 node for communication.
 */
class MuJoCoNode : public rclcpp::Node
{
    public:
            
        /**
         * @brief Contructor.
         * @param xmlLocation Where to find the XML file that defines the MuJoCo model.
         * @param jointStateTopicName The name of the topic to which joint state data is published.
         * @param jointControlTopicName The name of the topic to subscribe to for control inputs.
         * @param controlMode POSITION, VELOCITY, or TORQUE
         * @param simulationFrequency The rate in Hz to update the simulation
         * @param visualisationFrequency The rate with which to refresh the 3D rendering of the robot, environment
         */
        MuJoCoNode(const std::string &xmlLocation,
                   const std::string &jointStateTopicName,
                   const std::string &jointControlTopicName,
                   ControlMode controlMode = TORQUE,
                   int simulationFrequency = 500,
                   int visualizationFrequency = 20);
        
       /**
        * @brief Deconstructor.
        */
        ~MuJoCoNode();
        
        /**
         * @brief Sets the viewing properties in the window.
         * @param focalPoint Defines the x, y, z coordinate for the focus of the camera.
         * @param distance The distance from said focal point to the camera.
         * @param azimuth The angle of rotation around the focal point.
         * @param elevation The angle of the line of sight relative to the ground plane.
         * @param orthographic Type of projection. True for orthographics, false for perspective.
         */
        void
        set_camera_properties(const std::array<double,3> &focalPoint,
                              const double &distance = 2.0,
                              const double &azimuth = 140.0,
                              const double &elevation = -45.0,
                              const bool   &orthographic = false);
    private:

        std::vector<double> _torqueInput;                                                           ///< Used to store joint commands in torque mode
        
        ControlMode _controlMode;                                                                   ///< POSITION, VELOCITY, or TORQUE
        
        mjModel *_model;                                                                            ///< Underlying model of the robot.
        mjData  *_jointState;                                                                       ///< Joint state data (position, velocity, acceleration)

        mjvCamera  _camera;                                                                         ///< Camera for viewing
        mjvOption  _renderingOptions;                                                               ///< As it says
        mjvPerturb _perturbation;                                                                   ///< Allows manual interaction
        mjvScene   _scene;                                                                          ///< The environment that the robot is rendered in

        mjrContext _context;                                                                        ///< No idea what this does.

        GLFWwindow *_window;                                                                        ///< This displays the robot and environment.

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _jointStatePublisher;            ///< As it says on the label

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _jointCommandSubscriber;  ///< Subscriber for joint commands
        
        rclcpp::TimerBase::SharedPtr _simTimer, _visTimer;                                          ///< Regulates the ROS2 node

        sensor_msgs::msg::JointState _jointStateMessage;                                            ///< For publishing joint state data over ROS2
        
        int _simFrequency = 500;                                                                    ///< Speed at which the frequency runs
    
        bool _cameraOrthographic = false;                                                           ///< Sets the type of projection
        double _cameraAzimuth    = 45.0;                                                            ///< Rotation around camera focal point
        double _cameraDistance   = 2.5;                                                             ///< Distance from camera focal point
        double _cameraElevation  = -30;                                                             ///< Dictates height of camera
        std::vector<double> _cameraFocalPoint = {0.0, 0.0, 1.0};                                    ///< Where the camera is directed at
      
        /**
         * @brief Updates the robot state, publishes joint state information.
         */
        void
        update_simulation();
        
        /**
         * @brief Updates the visualisation.
         */
        void
        update_visualization();
        
        /**
         * @brief Callback function to handle incoming joint commands.
         * @param msg The message containing joint commands.
         */
        void
        joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

#endif
