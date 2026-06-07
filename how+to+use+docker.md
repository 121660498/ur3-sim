## 安装docker
follow the official guide to install docker: https://docs.docker.com/engine/install/ubuntu/

## 修改权限 docker permissions
  ```
  sudo groupadd docker
  sudo usermod -aG docker $USER 
  newgrp docker 
  ```


## 重建镜像（用于初次构建docker镜像，使用dockerfile文件，构建名为ros-noetic-base的docker镜像）
## 本镜像包括ubuntu22.04, ros noetic, 及git、国内源等相关工具
  ```
  cd /workspace/DockerSys
  docker build --no-cache --progress=plain -t ros-noetic-base -f dockerfile.md .
  ```

## 构建好镜像后，创建一个容器，并进入docker
  ```
  # 容器名为 ros_noetic_dev
  docker run -it --name ros_noetic_dev ros-noetic-base /bin/bash

  # 若要创建同名容器，并挂载宿主机目录 + GUI 支持，可以运行如下指令
  # 主要是将容器中/workspace 目录下的文件挂载在宿主机/home/gd/workspace 目录下，
  # 这样容器中文件修改后宿主机也同步修改  
  xhost +local:docker
  xhost +SI:localuser:用户名	仅授予指定Linux用户

  docker run -it --name ros_noetic_dev \
    --network host \
    --ipc host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:ro \
    -v /home/gd/workspace:/workspace \
    ros_noetic_base /bin/bash

  # !!!注意，挂载要在容器创建的时候设置，若已经创建容器，可以先保存容器内文件、删除旧容器，重新创建新容器，步骤如下：
  # 0 先做一次保险快照（可回滚）
  docker commit ros_noetic_dev ros-noetic-snapshot:before-mount
  # 1 宿主机创建持久化代码目录
  mkdir -p /home/gd/ros_ws
  # 2 可选：把旧容器里已有工作拷到宿主机（按你的实际路径改）
  docker cp ros_noetic_dev:/root/catkin_ws/. /home/gd/ros_ws/
  # 3 删除旧容器（镜像和快照还在）
  docker rm -f ros_noetic_dev
  # 4 重新创建同名容器，并挂载目录 + GUI 支持，参考前面

  ```

## 在宿主机上重新进入已经创建好的容器：
  # 查看现有容器名：
  ```
  docker ps -a --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
  # 有同名但 Exited
  docker start ros_noetic_dev
  docker exec -it ros_noetic_dev /bin/bash
  ```

## 进入容器进行开发
## 把容器当前状态保存成新镜像：docker commit <container> <new-image>:<tag>
  ```
  docker commit ros_noetic_dev ros-noetic-snapshot:latest
  ```

## 使用vs code 或者 cursor在宿主机编辑docker容器内的程序
# 初次打开
  ```
  # 0. 构建Dev container配置文件, cursor识别两种格式：
  #  .devcontainer/devcontainer.json
  #  .devcontainer/<配置名>/devcontainer.json
  #  我们新增/workspaces/DockerSys/.devcontainer/armcontrol/devcontainer.json

  #  然后按照下面的步骤打开：
  # 1. Ctrl+Shift+P
  # 2. 搜索 Dev Containers
  # 3. 选择 Dev Containers: Reopen in Container 或 Open Folder in Container
  # 4. 选择 armcontrol / ros_noetic_armcontrol

  ```
# 如果通过 Dev Containers 打开的（左下角有 >< 图标），下次直接用 VS Code 或者 cursor打开同一个工作区，点击左下角 Reopen in Container 即可自动进入。


## 常见命令对应
  # 看镜像：docker images
  # 看容器：docker ps -a
  # 用镜像创建容器：docker run ... <image>
  # 把容器当前状态保存成新镜像：docker commit <container> <new-image>:<tag>

<!-- 安装xclock -->
 apt-get update
 apt-get install -y x11-apps

 <!-- 启动ros -->
 roscore
 # 确保环境变量还在（如果在 devcontainer.json 配置好了就不需要手动 export）
rosrun rviz rviz

<!-- 安装git -->
apt-get update
apt-get install -y git





