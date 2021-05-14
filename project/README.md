# Realistic animation using Optimization and Linear Blend skinning 

This is the code for Geometric modelling course project. It can create realistic animations for a dog mesh based on motions generated for a quadruped robot.

# Environment
All the code was developed in Ubuntu 18.04 with gcc version 7.5.0 (Ubuntu 7.5.0-3ubuntu1~18.04) .

# Dependencies : 
1. Eigen
2. Pinocchio : installation link (https://stack-of-tasks.github.io/pinocchio/download.html)
    1.  sudo apt install -qqy lsb-release gnupg2 curl
    2.  echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
    3.  curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
    4.  sudo apt-get update
    5.  sudo apt install robotpkg-py36-pinocchio  # Adapt your desired python version here (only py36 or py27 is available)
    6.  Configure the path so that CMAKE can find the package. The configuration instructions are in the link - (https://stack-of-tasks.github.io/pinocchio/download.html). Please update it based on whether py27 or py36 is installed.
3. libigl

# Installation :
This project has the same structure as the libigl sample project. It expects the libigl header files to be located in the relative path provided in the CMAKE file. The procedure to build the project is as follows.

1. mkdir build
2. cd build
3. cmake ..
4. make 

Note : After installing pinocchio please source pinocchio so that CMAKE finds pinocchio while compiling the code. That is source /opt/openrobots/setup.bash or wherever you have stored the configuration files in the bash.

# Demos : 

To generate an animation run the "bbw" executable file. The corresponding source code can be found inside the demos folder.

# code organization :

1. bbw.cpp : contains the actual code that creates the animation based on the input mesh and motion
2. fk.cpp : forward kinematics file which updates the skeleton constraints based on the Robot URDF and input motion. This is used in the pre_draw function. 
3. motions directory : contains all the joint configurations generated from the motion planner.