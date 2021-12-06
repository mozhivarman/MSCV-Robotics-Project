

<p align="center">  
   <img src = "images/ub.png" width = 400>
</p >

# <p align="center">University of Burgundy
</p > 

# <p align="center">Master of Computer vision and Robotics</p >   
<p align="center">  
   <img src = "images/vibot.png" width = 80>
</p >

# <p align="center">Robotics Project</p> 

## <p align="center">Supervisors:</p > 
<p align="center"> Ralph SEULIN</p >   
<p align="center"> David FOFI </p >   
<p align="center"> Raphael DUVERNE </p >   
<p align="center"> Renato MARTINS </p >   
<p align="center"> Joaquin RODRIGUEZ </p >   

## <p align="center">Students:</p >

<p align="center">Shriarulmozhivarman GOBICHETTIPALAYAM</p>
<p align="center">Shriarulmozhivarman Gobichettipalayam</p>



## Table of contents:
- [AIM](#aim-of-the-project)
- [Introduction](#introduction)
- [Prior Knowledge](#prior-knowledge)
- [Tasks](#tasks)
   * [Image Calibration](#Image-Calibration)
   * [HSV Filtering](#HSV-Filtering)
   * [Cropping the Image](#Cropping-the-Image)
   * [Control](#Control)
- [Implementation](#implementation)
- [Setup](#setup)
- [Conclusions](#conclusions)
- [References](#references)



## AIM of the Project

The Aim of this project is to Demonstrate a Ground Robot (in our case its the [turtlebot3 burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) from [Robotis](https://emanual.robotis.com/))
performing Autonomous Driving by Perception (perform Lane Detection and Autonomus Driving of the robot).The project has to done with the [ROS Melodic](http://wiki.ros.org/melodic).

## Introduction

The Robot Operating System (ROS) is an open-source robotics middleware suite and a framework for robot software development, providing operating system-like functionality.ROS provides standard operating system services such as hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management.

TurtleBot3(Burger) is a programmable ROS-based mobile robot used in research and education.The turtlebot3 burger has the sensor such as a [360 Laser Distance Sensor](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/) and a [Raspberry Pi Camera Module](https://www.raspberrypi.com/products/camera-module-v2/), control borads such as [OpenCR1.0](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_opencr1_0/) (for low level control) and a Single Board Computer  [Raspberry Pi 3B+](https://www.raspberrypi.com/products/raspberry-pi-3-model-b-plus/)(for high level control and running ros Nodes).


**Autonomous Driving by Perception** with the turtlebot3 burger on the [autorace](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving_autorace/) track can be done with the camera. The image caputred from the camera has to be processed by the Remote PC and make the robot autonomous Drive through the auto race track.


## Prior Knowledge
The lane has to follow the lanes with ***yellow*** on the left and ***white*** on the right and the distance between the lane remains constant throught out the track(even in turns and in the tunnel).

<p align="center
">  
   <img src = "images/lane1.jpg" width = 200 height = 200>
</p >

The has to autonomous drive in between the lane with this prior knowledge.

## Tasks
- Image Calibration 

The Image from the raspicam has to be calibrated, so the image obtained will be contrast, sharper image of the lanes for us to filter using the HSV color space.

- HSV Filtering

The Image must be filted to get the ***yellow*** and ***white*** lane in the image. The filtering can be done in the HSV color Space rather than the RGB color Space because the color can be seperated in variations in the image color values due to various lightening conditions, shadows in HSV color spaceeasily.
- Cropping the Image

The whole image from the image is too large to process and the we do no need the whole image to navigate, so the image has to croppe to right infornt of the robot to naviaget the track autonomously. 

- Control

The robot must maintain in the ***middle*** of the track to navigate the track.By cropping and filtering the whole image, we can estimate the pose of the robot with respect to the track from the image, with this pose and image we can get the error in the position of the robot in the image and apply velocity commands according to the error from the image.


## Implementation
## Setup
## Conclusions
## References