# robot-ultrasound-SLAM
Masters Capstone Project: Enhancing Robotic Ultrasound Imaging with ElasticFusion SLAM and Pose Graph Optimization​

# Project Documentation

Welcome to the project repository! Here you will find the necessary information to understand, set up, and run the project. 

![Experimental Setup](https://github.com/shivasurya1999/robot-ultrasound-SLAM/blob/master/US_setup_lab.jpeg?raw=true)

## Detailed Project Approach

For a comprehensive explanation of the methodologies employed throughout this project, please refer to the detailed report:

- [Capstone Project Report](slolla_Capstone_Report.pdf)

## Running the System with Docker

This project utilizes rosbag files which can be run on Ubuntu 22.04 leveraging Docker to simulate real-world data collection and processing. The data in the rosbag file includes rgb images, depth images, groundtruth data, ultrasound images and their respective timestamps. The process for setting up, running the system, and obtaining results is fully automated. For step-by-step instructions on how to run this setup, please see:

- [INSTRUCTIONS TO RUN ElasticFusion](INSTRUCTIONS%20TO%20RUN%20ElasticFusion)

## Trajectory Analysis with Evo Package

The `evo` package is utilized for analyzing the output trajectory data obtained from the system alongside the groundtruth data. This analysis helps in validating the accuracy and effectiveness of the SLAM implementations used in the project.

For more information on how to perform these analyses, refer to the instruction document linked above.

![ElasticFusion Results](https://github.com/shivasurya1999/robot-ultrasound-SLAM/blob/master/6bf194d0-d419-4c69-9f6f-c78cdef0c0b8.gif?raw=true)
