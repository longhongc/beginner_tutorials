# Beginner Tutorials for ROS2
This repository contains following practices for ROS2
- [**Publisher subscriber**](#publisher-and-subscriber)  

## Install
Clone the repository to your ros2 workspace src folder
```
cd {ros2_ws}/src
git clone https://github.com/longhongc/beginner_tutorials.git
```

## Build
Source your ros2 workspace  
Setup.bash can be .zsh, depends on your shell
```
source {ros2_ws}/install/setup.bash 
cd {ros2_ws}
colcon build
```

## Publisher and Subscriber
### Run

Run publisher and listener on different terminals
```
ros2 run beginner_tutorials talker
ros2 run beginner_tutorials listener
```
### Result
![simple_pub_sub](https://user-images.githubusercontent.com/28807825/200461430-fd63491a-4e3d-4c6a-a7dc-0f1cc3ca5a7b.png)
