# 实验教程

## 创建工作空间

```bash
      cd ~/    # 切换到主目录文件夹
      mkdir -p catkin_ws/src  # 创建工作空间，src用于保存功能包和代码
      cd ~/catkin_ws # 切换到工作空间目录下
      catkin_make # ros编译，编译后会生成一系列编译文件
```
## 下载代码到本地

```bash
cd ~/catkin_ws/src
git clone https://github.com/zhouhandsome/Robot_intelligent_technology.git

```
安装依赖项
```bash
cd ~/catkin_ws/src/Robot_intelligent_technology/code
sh ./install_for_noetic.sh
```
## 编译

```bash
cd ~/catkin_ws
catkin_make
```
## 查看机器人

```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
roslaunch mrobot_description my_robot.launch
```
## 查看gazebo仿真环境

```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
roslaunch mrobot_description gazebo_test.launch
```
## slam建图
### 启动仿真环境和slam建图节点
```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
roslaunch mrobot_description slam_map.launch
```
### 启动键盘控制节点

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

```
## 导航
```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
roslaunch mrobot_description robot_navigation.launch
```