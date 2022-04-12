# RWA3_ENPM809E
Baseline package for RWA3.

# Disclaimer

This package is provided as an example on how to perform kitting with the kitting robot. There is no guarantee that it will work everytime you run the demo.

# Installation

Assuming your catkin workspace is `catkin_ws`
```bash
cd ~/catkin_ws/src
git clone https://github.com/zeidk/RWA3_ENPM809E.git
cd ..
rosdep install --from-paths ./src --ignore-packages-from-source -y
catkin build
source ~/catkin_ws/setup.bash
```

# Run

- To run the C++ code

```bash
roslaunch RWA3_ENPM809E ariac.launch
rosrun RWA3_ENPM809E kitting_movegroup_node
```

- To run the Python code

```bash
roslaunch RWA3_ENPM809E ariac.launch
rosrun RWA3_ENPM809E kitting_commander_node
```