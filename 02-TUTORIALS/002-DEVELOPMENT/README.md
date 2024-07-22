# ROS2 Humble Development

## ROS Programming Structure
```bash
workspace_folder/
    src/
      cpp_package/
          CMakeLists.txt
          include/cpp_package/
          package.xml
          src/

      py_package/
          package.xml
          resource/py_package
          setup.cfg
          setup.py
          py_package/
      ...
```

## ROS2 Workspace
A ROS Workspace is a directory where you organize and build your ROS projects. It typically follows a standard structure that includes subdirectories for storing source code, build files, and other resources. The workspace is used to manage multiple ROS packages, allowing for modular development and easy dependency management.

Creating and Using a ROS Workspace
```bash
# Create a Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build Package:
colcon build

# Source the Workspace:
source install/setup.bash
```

## ROS Packages
A ROS Package is a fundamental unit of software organization in ROS. Each package contains the software and resources needed to implement specific functionalities, such as nodes, libraries, datasets, configuration files, and third-party dependencies. Packages are designed to be modular and reusable.

```bash
# Creating a ROS Package:
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake package_name
cd ~/ros2_ws

# Build Package:
colcon build

# Source the Workspace:
source install/setup.bash
```

In summary, a ROS Workspace is an environment that contains and organizes your ROS packages, while ROS Packages are the individual units within the workspace that contain the code and resources for specific functionalities.


# PLAN!!! (nnt dihapus)
### Tutor Content: pub-sub (v), server-client (v), interfaces (v), params, plugins, actions, launch file
### Next: TF2, urdf, RViz2, Gazebo, SLAM TOOLBOX, NAVIGATION STACK, OpenCV