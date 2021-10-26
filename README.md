# Intro

Hi everyone! We personally prefer not to use ROS in our systems, because what is life without a little risk and throwing your compute out the window because the documentation for what you need is non-existent. This implementation is written for C++ and CMake.

This is a minimum case use of cartographer 2D SLAM (for now), as the documentation is not very clear. We created this repo to be a starting point (also for us). 

# Requirements

First of all, you have to follow the steps from the Cartographer documentation to intall it (without ROS): https://google-cartographer.readthedocs.io/en/latest/index.html
You will have to install yaml-cpp too from: https://github.com/jbeder/yaml-cpp

# What did we do

There are 2 mains (for now): yaml and lua implementation

In the main script you will find the implementation of a SLAM execution using a fake trajectory. You can modify the yamls or lua files to change parameters. 
To execute the example, clone the repo and then: 
```
cd /path/to/cartographer_without_ros
mkdir build && cd build
cmake .. && make -j4
./fake_trajectory_yaml # or ./fake_trajectory_lua
```

# WIP

- 3D SLAM
- Implementation with Lidar
- Few variables still missing for config but for now I do not need them so I will modify it as I go
- Why does it die when finishing execution?

