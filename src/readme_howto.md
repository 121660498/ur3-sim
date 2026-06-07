## 下载代码
mkdir -p ur3_sim/src
cd ur3_sim/src
apt-get update
apt-get install -y git
git clone https://github.com/cambel/ur3.git

## 安装依赖
rosinstall . /opt/ros/noetic /workspaces/Workspace/ur3_sim/src/dependencies.rosinstall
rosdepc update
rosdepc install --from-paths src --ignore-src --rosdistro=noetic -y

## 编译
## 如果没有moveit，注意安装
sudo apt update
sudo apt install ros-noetic-moveit
sudo apt install swig
sudo apt install libnlopt-dev libnlopt-cxx-dev libnlopt0

sudo apt-get install -y ros-noetic-effort-controllers
sudo apt-get install -y ros-noetic-industrial-robot-status-interface
sudo apt-get install -y ros-noetic-scaled-joint-trajectory-controller
sudo apt-get install -y ros-noetic-gripper-action-controller
sudo apt-get install -y ros-noetic-speed-scaling-interface
sudo apt-get install -y ros-noetic-speed-scaling-state-controller
sudo apt-get install -y ros-noetic-pass-through-controllers
sudo apt-get install -y ros-noetic-ur-client-library


sudo apt-get update && sudo apt-get install -y \
ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-controller-manager ros-noetic-controller-stopper \
ros-noetic-joint-state-controller ros-noetic-joint-trajectory-controller ros-noetic-effort-controllers \
ros-noetic-position-controllers ros-noetic-velocity-controllers ros-noetic-gripper-action-controller \
ros-noetic-ur-client-library ros-noetic-ur-msgs ros-noetic-scaled-joint-trajectory-controller \
ros-noetic-speed-scaling-interface ros-noetic-speed-scaling-state-controller ros-noetic-pass-through-controllers \
ros-noetic-industrial-robot-status-interface ros-noetic-industrial-core ros-noetic-gazebo-ros-control \
ros-noetic-gazebo-plugins ros-noetic-moveit ros-noetic-moveit-visual-tools ros-noetic-tf2-geometry-msgs \
ros-noetic-tf2-sensor-msgs x11-apps mesa-utils git

source /opt/ros/noetic/setup.bash

rm -rf build/ devel/
catkin_make
source devel/setup.bash

## 查看带夹爪的机器人模型
roslaunch ur_gripper_description display_with_gripper_hande.launch ur_robot:=ur3

## 打开仿真器
roslaunch ur_gripper_gazebo ur_gripper_85_cubes.launch ur_robot:=ur3 grasp_plugin:=1
# 如果打不开gazebo，大概率是权限的问题
# 宿主机终端执行
xhost +local:root
# 然后在当前容器/终端里执行：
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
source /workspace/ur3_sim/devel/setup.bash
roslaunch ur_gripper_gazebo ur3_cubes_example.launch

# 如果还是打不开，可能上次gazebo没有正确退出
# 先清掉旧进程：
pkill -f roslaunch
pkill -f gzserver
pkill -f gzclient
pkill -f rosmaster
pkill -f controller_manager
# 确认没有残留：
ps -ef | grep -E "roslaunch|rosmaster|gzserver|gzclient|controller_manager"

## 开启控制器
rosrun ur_control controller_examples.py -m
# 可能提示需要安装python
sudo apt install python-is-python3
# quaternion
python3 -m pip install numpy-quaternion
python3 -m pip install pyquaternion





