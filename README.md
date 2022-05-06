# rom2109 Autonomous Mobile Robot simulation

> cd your_dir/catkin_ws/src

# clone teleop package.
> sudo apt-get install ros-noetic-teleop-twist-keyboard

# install gazebo ros packages.
> sudo apt-get install ros-noetic-ros-control
> sudo apt-get install ros-noetic-ros-controllers
> sudo apt-get install ros-noetic-gazebo-ros-pkgs

# install joint state publisher gui.
> sudo apt-get install ros-noetic-joint-state-publisher
> sudo apt-get install ros-noetic-joint-state-publisher-gui

# clone hector gazebo model.
> git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git -b melodic-devel

# clone ros navigation package.
> sudo apt-get install ros-noetic-navigation

> cd ~/catkin_ws
> catkin_make

Now, we can install Evarobot ROS Noetic packages for PC.

> cd ~/catkin_ws/src
> git clone -b noetic-devel git@github.com:ROM-robotics/rom2109_simulation.git

> cd ~/catkin_ws
> catkin_make