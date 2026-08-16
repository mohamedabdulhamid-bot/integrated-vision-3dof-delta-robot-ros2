## Integrated Mechatronic System for an Automated Foam Plate Production Line and Delta Robotic Packaging

    Mechatronics Engineering Senior Graduation Project

    A fully custom-built 3-DOF Delta robot bridging mechanical design, embedded control, and high-level robotics software for automated industrial packaging.

<img width="666" height="374" alt="Pasted image (2)" src="https://github.com/user-attachments/assets/4db824f6-69fc-44af-ad6f-183db621ddfa" />



 ## Overview

This repository contains the complete hardware and software architecture for our senior graduation project. Designed and manufactured entirely from scratch, this system operates as an autonomous packaging unit. It utilizes ESP32 microcontrollers for real-time hardware execution and a ROS 2 architecture for inverse kinematics. Combined with an OpenCV computer vision pipeline, the robot is capable of real-time object detection, dynamic pick-and-place operations, and custom trajectory generation.
 Tech Stack & Hardware

    Operating System: Ubuntu Linux

    Software Framework: ROS 2 (Jazzy)

    Languages: Python, C/C++

    Computer Vision: OpenCV (HSV color thresholding, contour extraction, shape detection)

    Embedded Hardware: ESP32 (running FreeRTOS)

    Actuation & Sensing: DC gear motors, Cytron motor drivers, quadrature encoders

    Mechanical Design: SolidWorks

## presentation
 [click here to see the presentation on google drive](https://drive.google.com/drive/folders/1eVsxZofNP7hxiP0DvLmWnte0NTw9Deb-?usp=sharing)

## Repository Structure

This repository is organized into distinct subsystems to separate mechanical, embedded, and high-level software components:

    cad_models/ - Contains the SolidWorks mechanical designs and assemblies.

    esp32_firmware/ - C/C++ source code for the microcontrollers handling PID control loops, encoder reading, and motor actuation.

    ros2_ws/ - The ROS 2 workspace containing custom packages:

        master_brain_node

        delta_kinematics

        trajectory_planner

        serial_bridge_node (Facilitates ROS 2 to ESP32 communication)

        delta_description (URDF and RViz visualization)

    vision_pipeline/ - OpenCV Python scripts for the overhead camera and shape detection algorithms.

    presentation/ - Project defense slides and official documentation.

## Getting Started
Prerequisites

    Ubuntu Linux installed

    ROS 2 Jazzy Desktop installed

    Python 3 and OpenCV installed

Build Instructions
Bash

# Clone the repository
git clone https://github.com/mohamedabdulhamid-bot/integrated-vision-3dof-delta-robot-ros2.git

# Navigate to the ROS 2 workspace
cd integrated-vision-3dof-delta-robot-ros2/ros2_ws

# Build the packages
colcon build

# Source the environment
source install/setup.bash

## Project Team

This system was designed and developed by a dedicated team of Mechatronics Engineering graduates from Assiut University:

    Mohamed Ahmed Abdulhamid Abdellah (Team Leader)

    Ismaeel Abdelnasser

    Belal Gamal

    Mahmoud Mamdouh El-Mamlouk

    Mahmoud Abdel-Raouf Marei

    Yahia Abdulhakim Hammadi Al-Nubi

    Mahmoud Abdelsamie
    

 Acknowledgments & Funding

This graduation project was officially supported and funded by the Information Technology Academia Collaboration (ITAC) program.

---
Author: Mohamed Ahmed Abdulhamid Abdellah

Contact: mohamed.abdulhamid404@gmail.com
