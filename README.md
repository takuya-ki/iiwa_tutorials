# iiwa_tutorials

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License: BSD](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![repo size](https://img.shields.io/github/repo-size/takuya-ki/iiwa_tutorials)

- ROS package for KUKA LBR iiwa 14 R820 tutorial.
- Docker for simulation and control environments for KUKA LBR iiwa 14 R820.

## Dependencies

### Docker build environments

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=22.04+LTS)
  - Docker 26.1.1
  - Docker Compose 2.27.0

### LBR iiwa 14 R820

- [Ubuntu 18.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=18.04+LTS)
  - [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
  - [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack)
  - [Byobu](https://www.byobu.org/)
- [LBR iiwa](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/lbr-iiwa) 

## Installation

1. Follow [iiwa_stack wiki](https://github.com/IFL-CAMP/iiwa_stack/wiki) to setup a package with Sunrise Workbench  
  <img src=image/StationSetup.cat.jpg width=350>  
  <img src=image/RoboticsAPI.data.xml.jpg width=350>  
2. Connect an Ethernet cable between the host computer and the Ethernet port of controller  
  <img src=image/X66.jpg width=150>  
3. Set the network configuration as below  
    <img src=image/network.png width=280>  
    - The ros node expects to reach the robot at the IP `172.31.1.147`  
    - The ROS nodes on the robot side expects to reach the ROS master at the IP `172.31.1.150`  
    - This is set with the pendant as below  
    <img src=image/pendant_network.jpg width=250>  
    - The ROS_IP and ROS_MASTER_URI must be set as below in .bashrc  
    <img src=image/bashrc.png width=320>
4. Build the docker environment as below  
    ```bash
    sudo apt install byobu && git clone git@github.com:takuya-ki/iiwa_tutorials.git --depth 1 && cd iiwa_tutorials && COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel  
    ```

## Usage with docker

### Using utility scripts

1. Build and run the docker environment
- Create and start docker containers in the initially opened terminal
  ```bash
  docker compose up
  ```

2. ```roscore``` on the host machine
  - This application requires having a ROS Master running on the ROS machine connected to the cabinet. ([iiwa_stack wiki](https://github.com/IFL-CAMP/iiwa_stack/wiki))

### Simulation

3. Run a demonstration on the host machine  

- Executing a pick-and-place demonstration
  ```bash
  ./utils/iiwa_pp_fake.sh
  ```
  - Initialize the eef pose
    ```bash
    initialize
    ```
  - Demonstrate pick-and-place of a object
    ```bash
    pp
    ```

### Real robot

3. Run the application ROSSmartServo on the KUKA Smartpad  
4. Run a demonstration on the host machine  

- Testing the moveit with GUI
  ```bash
  ./utils/iiwa_moveit.sh
  ```

- Executing a simple motion
  ```bash
  ./utils/iiwa_singlecomm.sh
  ```

- Executing a repetitive motion
  ```bash
  ./utils/iiwa_loop.sh
  ```

- Executing a repetitive linear motion
  ```bash
  ./utils/iiwa_looplin.sh
  ```

- Executing a repetitive motion with ROS Action
  ```bash
  ./utils/iiwa_loopact.sh
  ```

- Executing a wiggle demonstration
  ```bash
  ./utils/iiwa_wiggle.sh
  ```

- Executing a pick-and-place demonstration
  ```bash
  ./utils/iiwa_pp.sh
  ```
  - Initialize the eef pose
    ```bash
    initialize
    ```
  - Demonstrate pick-and-place of a object
    ```bash
    pp
    ```

### Successfull termination

1. Terminate the demonstration
2. Stop the application ROSSmartServo on the pendant
3. In the end, kill the roscore process if you need

### Manually execute commands

1. Build and run the docker environment
- Create and start docker containers in the initially opened terminal
  ```bash
  docker compose up
  ```
- Execute the container in another terminal
  ```bash
  xhost + && docker exec -it iiwa_container bash
  ```

2. Run a demonstration in the container  
    ```bash
    byobu
    ```
    - First command & F2 to create a new window & Second command ...
    - Ctrl + F6 to close the selected window

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)  

We always welcome collaborators!

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
