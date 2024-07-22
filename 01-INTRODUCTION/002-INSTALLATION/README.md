# ROS2 Humble Installation

## Install ROS on Ubuntu (Debian Packages)
* System Requirements
    * Ubuntu Linux - Jammy (22.04) 64-bit
    * At least 5GB free space

### 1. Locale Configuration
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### 2. Software Properties and Universe Repository
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### 3. Curl Installation and ROS Key
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### 4. ROS2 Sources Configuration
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 5. Update and Upgrade
```bash
sudo apt update
sudo apt upgrade
```

### 6. ROS2 Humble Full Installation
```bash
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

### 7. Setup Environment
```bash
source /opt/ros/humble/setup.bash # for each terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc # just once
```

### 8. Verify Installation
```bash
ros2
```

### 9. Additional Setup (colcon build)
```bash
sudo apt install python3-colcon-common-extensions
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
```