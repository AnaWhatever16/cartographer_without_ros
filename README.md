# Intro

Hi everyone! I personally prefer not to use ROS in my systems, because what is life without a little risk and throwing your compute out the window because the documentation for what you need is non-existent.
This implementation is fully in C++ and CMake and I used YAML instead of Lua files for configuration purposes because I was unable to identify how to use the lua files properly.

# Requirements

First of all, you have to follow the steps from the Cartographer documentation to intall it (without ROS): https://google-cartographer.readthedocs.io/en/latest/index.html
You will have to install yaml-cpp too from: https://github.com/jbeder/yaml-cpp

# What did I do

In the main script you will find the implementation of a SLAM execution using a fake trajectory. You can modify the yamls to change parameters. 

# WIP

- Implementation with Lidar
- Few variables still missing for config but for now I do not need them so I will modify it as I go

