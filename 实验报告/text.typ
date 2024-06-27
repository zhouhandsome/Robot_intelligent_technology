#set page(
     paper: "us-letter",
     margin: (x: 3.0cm, y: 1.5cm),
    //  footer: counter(heading).display()
     numbering: "1"
)
#set par(
     justify: true,
     leading: 0.6em,//行间距
    //  linebreaks: 0.5em,
    first-line-indent: 1em
)
#outline(
  title: [目录],
  indent: 2em,
)
#pagebreak()
// par(
// leading: length,
// justify: bool,
// linebreaks: autostr,
// first-line-indent: length,
// hanging-indent: length,
// content,
// ) -> content
#let title() ={
set align(center)
strong[= 基于ROS的移动机器人路径规划与自主导航仿真研究]
}
#title()

== 一、实验目的和要求

=== 1、实验目的
\

（1）ROS环境熟练掌握：熟悉ROS（Robot Operating System）框架及其在机器人软件开发中的应用，包括节点、消息、服务和参数等基本概念。

（2）仿真环境搭建：在ROS中搭建Gazebo或Rviz仿真环境，用于模拟移动机器人的物理行为和环境交互，包括机器人模型导入、传感器配置与环境设置。

（3）理论知识实践：将路径规划和自主导航的理论知识应用于实践，通过编写ROS节点实现环境感知、地图创建、路径规划及运动控制等功能。

（4）算法仿真验证：在ROS/Gazebo或Rviz平台上实现路径规划算法（如A\*和DWA）的仿真，评估算法在复杂环境中的性能和鲁棒性。

（5）系统集成能力提升：整合各个模块，形成一个完整的基于ROS的自主导航系统，并在仿真环境中验证系统的功能完整性与稳定性。

=== 2、实验要求
\

ROS环境配置：正确安装并配置ROS及Gazebo仿真环境，确保机器人模型、传感器模型能正常运行于仿真场景中。

传感器数据处理：通过ROS话题（topics）订阅激光雷达或摄像头等传感器数据，处理并应用于环境感知。

SLAM实施：在Gazebo仿真中实现SLAM，创建并维护环境地图，确保地图的实时性和准确性。

路径规划算法开发：使用ROS中的Navigation Stack或自定义节点实现A或其它全局规划算法。
结合DWA（动态窗口算法）等局部规划方法，实现实时避障和路径跟踪。

控制系统设计与实现：设计控制逻辑，编写ROS节点控制机器人按照规划路径行驶，考虑运动学模型和动力学限制。

仿真测试与评估：设计并执行一系列仿真测试场景，分析实验结果和数据并得出有效结论。

报告与演示：撰写详细实验报告，总结实验过程、分析结果、遇到的问题及解决方案，并准备实验成果的演示材料。

== 二、开发环境及软件工具
\

操作系统： Windows 10、Ubuntu20.04LTS

软件：Python、Jupter notebook、VMware workstation、ROS、Gazebo、Rviz、Pip、Conda、Visual Studio Code

== 三、实验原理和设计方案

=== 1、实验原理

==== 1.1 Ros操作系统介绍
\

ROS是一个适用于机器人的开源的元操作系统。它提供了操作系统应有的服务，包括硬件抽象，底层设备控制，常用函数的实现，进程间消息传递，以及包管理。它也提供用于获取、编译、编写、和跨计算机运行代码所需的工具和库函数。它的目的是为了提高机器人研发效率。
==== 1.2 Ros使用基本方法
\
ros中的通信机制

话题通信实现模型是比较复杂的，该模型如下图所示,该模型中涉及到三个角色:
```
          ROS Master (管理者)
          Talker (发布者)
          Listener (订阅者)
```
ROS Master 负责保管 Talker 和 Listener 注册的信息，并匹配话题相同的 Talker 与 Listener，帮助 Talker 与 Listener 建立连接，连接建立后，Talker 可以发布消息，且发布的消息会被 Listener 订阅。
#image(".\image\3-1\01话题通信模型.jpg",width: 100%,)

服务通信较之于话题通信更简单些，理论模型如下图所示，该模型中涉及到三个角色:

```
          ROS master(管理者)
          Server(服务端)
          Client(客户端)
```
ROS Master 负责保管 Server 和 Client 注册的信息，并匹配话题相同的 Server 与 Client ，帮助 Server 与 Client 建立连接，连接建立后，Client 发送请求信息，Server 返回响应信息。
#image(".\image\3-1\02_服务通信模型.jpg",width: 100%,)
==== 1.3 Gazebo和rviz机器人仿真平台介绍

===== 1.3.1 URDF
\

URDF是 Unified Robot Description Format 的首字母缩写，直译为统一(标准化)机器人描述格式，可以以一种 XML 的方式描述机器人的部分结构，比如底盘、摄像头、激光雷达、机械臂以及不同关节的自由度.....,该文件可以被 C++ 内置的解释器转换成可视化的机器人模型，是 ROS 中实现机器人仿真的重要组件
===== 1.3.2 rviz
\

rviz 是 ROS Visualization Tool 的首字母缩写，直译为ROS的三维可视化工具。它的主要目的是以三维方式显示ROS消息，可以将 数据进行可视化表达。例如:可以显示机器人模型，可以无需编程就能表达激光测距仪（LRF）传感器中的传感 器到障碍物的距离，RealSense、Kinect或Xtion等三维距离传感器的点云数据（PCD， Point Cloud Data），从相机获取的图像值等

以“ros- [ROS_DISTRO] -desktop-full”命令安装ROS时，RViz会默认被安装。

运行使用命令rviz或rosrun rviz rviz

如果rviz没有安装，请调用如下命令自行安装:

sudo apt install ros-[ROS_DISTRO]-rviz
===== 1.3.3 gazebo
\

Gazebo是一款3D动态模拟器，用于显示机器人模型并创建仿真环境,能够在复杂的室内和室外环境中准确有效地模拟机器人。与游戏引擎提供高保真度的视觉模拟类似，Gazebo提供高保真度的物理模拟，其提供一整套传感器模型，以及对用户和程序非常友好的交互方式。

以“ros- [ROS_DISTRO] -desktop-full”命令安装ROS时，gzebo会默认被安装。

运行使用命令gazebo或rosrun gazebo_ros gazebo
==== 1.4 SLAM
\

===== 1.4.1 SLAM步骤 
SLAM（Simultaneous Localization and Mapping）是一种同时进行自主定位和环境地图构建的技术。其基本过程如下：

初始定位：在没有先验地图的情况下，机器人首先需要进行初始定位，即确定自身在未知环境中的初始位置和方向。

运动预测：机器人根据传感器获取的运动信息（例如里程计、惯性测量单元等）预测下一时刻自身的位置和方向。

数据获取：机器人通过各种传感器（例如相机、激光雷达、RGB-D摄像头等）获取环境的感知数据。

特征提取与匹配：对于每帧感知数据，机器人通过特征提取算法（例如角点、边缘等）提取关键特征，并和先前帧的特征进行匹配。

建图：通过匹配得到的特征点，机器人从运动中提取出环境的空间结构信息，逐步构建地图。

数据关联：将当前观测到的特征点与已有地图进行关联，更新地图中的特征点位置信息。

优化：利用非线性优化方法（例如图优化算法）以最小化定位误差和地图重建误差，同时优化机器人的轨迹和地图。

循环检测与闭环校正：通过检测到已经遍历过的区域或者相似的特征，识别出环路闭合的情况，进行地图的校正和修正。

定位更新：通过循环检测和闭环校正，对机器人的定位进行更新和修正，提高定位的准确性和鲁棒性。

持续循环：重复执行上述步骤，连续地进行感知、定位和建图，以不断改进地图的精度和机器人的定位能力。

通过以上步骤，SLAM系统可以在未知环境中实现同时定位和建图，为自主导航和路径规划等任务提供准确的环境信息。
===== 1.4.2 ROS中gmapping功能包介绍
SLAM算法有多种，当前我们选用gmapping。

1.gmapping简介

gmapping 是ROS开源社区中较为常用且比较成熟的SLAM算法之一，gmapping可以根据移动机器人里程计数据和激光雷达数据来绘制二维的栅格地图，对应的，gmapping对硬件也有一定的要求:

该移动机器人可以发布里程计消息
机器人需要发布雷达消息(该消息可以通过水平固定安装的雷达发布，或者也可以将深度相机消息转换成雷达消息)
关于里程计与雷达数据，仿真环境中可以正常获取的，不再赘述，栅格地图如案例所示。

gmapping 安装前面也有介绍，命令如下:

sudo apt install ros-<ROS版本>-gmapping

2.gmapping节点说明

gmapping 功能包中的核心节点是:slam_gmapping。为了方便调用，需要先了解该节点订阅的话题、发布的话题、服务以及相关参数。

2.1订阅的Topic

- tf (tf/tfMessage)用于雷达、底盘与里程计之间的坐标变换消息。
- scan(sensor_msgs/LaserScan)SLAM所需的雷达信息。

2.2发布的Topic

- map_metadata(nav_msgs/MapMetaData)地图元数据，包括地图的宽度、高度、分辨率等，该消息会固定更新。
- map(nav_msgs/OccupancyGrid)地图栅格数据，一般会在rviz中以图形化的方式显示。
- entropy(std_msgs/Float64)机器人姿态分布熵估计(值越大，不确定性越大)。

2.3服务

- dynamic_map(nav_msgs/GetMap)用于获取地图数据。

2.4参数

- base_frame(string, default:"base_link")机器人基坐标系。
- map_frame(string, default:"map")地图坐标系。
- odom_frame(string, default:"odom")里程计坐标系。
- map_update_interval(float, default: 5.0)地图更新频率，根据指定的值设计更新间隔。
- maxUrange(float, default: 80.0)激光探测的最大可用范围(超出此阈值，被截断)。
- maxRange(float)激光探测的最大范围。
.... 参数较多，上述是几个较为常用的参数，其他参数介绍可参考官网。

2.5所需的坐标变换

- 雷达坐标系→基坐标系:一般由 robot_state_publisher 或 static_transform_publisher 发布。

- 基坐标系→里程计坐标系:一般由里程计节点发布。

2.6发布的坐标变换

- 地图坐标系→里程计坐标系:地图到里程计坐标系之间的变换。
==== 1.5 路径规划实现（局部地图，全局地图，碰撞算法，规划算法）
\

毋庸置疑的，路径规划是导航中的核心功能之一，在ROS的导航功能包集navigation中提供了 move_base 功能包，用于实现此功能。

===== (1) move_base简介

move_base 功能包提供了基于动作(action)的路径规划实现，move_base 可以根据给定的目标点，控制机器人底盘运动至目标位置，并且在运动过程中会连续反馈机器人自身的姿态与目标点的状态信息。move_base主要由全局路径规划与本地路径规划组成。

move_base已经被集成到了navigation包，navigation安装前面也有介绍，命令如下:

sudo apt install ros-<ROS版本>-navigation

===== (2)move_base节点说明

move_base功能包中的核心节点是:move_base。为了方便调用，需要先了解该节点action、订阅的话题、发布的话题、服务以及相关参数。

2.1动作

动作订阅
```bash
    move_base/goal(move_base_msgs/MoveBaseActionGoal)===>move_base 的运动规划目标。
    move_base/cancel(actionlib_msgs/GoalID)===>取消目标。
```

动作发布
```bash
  move_base/feedback(move_base_msgs/MoveBaseActionFeedback)===>连续反馈的信息，包含机器人底盘坐标。
  move_base/status(actionlib_msgs/GoalStatusArray)===>发送到move_base的目标状态信息。
  move_base/result(move_base_msgs/MoveBaseActionResult)===>操作结果(此处为空)。
```
\

2.2订阅的Topic

- move_base_simple/goal(geometry_msgs/PoseStamped)运动规划目标(与action相比，没有连续反馈，无法追踪机器人执行状态)。
\ 

2.3发布的Topic

- cmd_vel(geometry_msgs/Twist)输出到机器人底盘的运动控制消息。
\

2.4服务

- make_plan(nav_msgs/GetPlan)请求该服务，可以获取给定目标的规划路径，但是并不执行该路径规划。

- clear_unknown_space(std_srvs/Empty)允许用户直接清除机器人周围的未知空间。

- clear_costmaps(std_srvs/Empty)允许清除代价地图中的障碍物，可能会导致机器人与障碍物碰撞，请慎用。
\
===== (3)move_base与代价地图

3.1概念

机器人导航(尤其是路径规划模块)是依赖于地图的，地图在SLAM时已经有所介绍了，ROS中的地图其实就是一张图片，这张图片有宽度、高度、分辨率等元数据，在图片中使用灰度值来表示障碍物存在的概率。不过SLAM构建的地图在导航中是不可以直接使用的，因为：

SLAM构建的地图是静态地图，而导航过程中，障碍物信息是可变的，可能障碍物被移走了，也可能添加了新的障碍物，导航中需要时时的获取障碍物信息；
在靠近障碍物边缘时，虽然此处是空闲区域，但是机器人在进入该区域后可能由于其他一些因素，比如：惯性、或者不规则形体的机器人转弯时可能会与障碍物产生碰撞，安全起见，最好在地图的障碍物边缘设置警戒区，尽量禁止机器人进入...
所以，静态地图无法直接应用于导航，其基础之上需要添加一些辅助信息的地图，比如时时获取的障碍物数据，基于静态地图添加的膨胀区等数据。

3.2组成

代价地图有两张:global_costmap(全局代价地图) 和 local_costmap(本地代价地图)，前者用于全局路径规划，后者用于本地路径规划。

两张代价地图都可以多层叠加,一般有以下层级:

Static Map Layer：静态地图层，SLAM构建的静态地图。

Obstacle Map Layer：障碍地图层，传感器感知的障碍物信息。

Inflation Layer：膨胀层，在以上两层地图上进行膨胀（向外扩张），以避免机器人的外壳会撞上障碍物。

Other Layers：自定义costmap。

多个layer可以按需自由搭配。



3.3碰撞算法

在ROS中，如何计算代价值呢？请看下图:

#image(".\image\3-1\碰撞算法.jpg"),
 


上图中，横轴是距离机器人中心的距离，纵轴是代价地图中栅格的灰度值。

- 致命障碍:栅格值为254，此时障碍物与机器人中心重叠，必然发生碰撞；
- 内切障碍:栅格值为253，此时障碍物处于机器人的内切圆内，必然发生碰撞；
- 外切障碍:栅格值为[128,252]，此时障碍物处于其机器人的外切圆内，处于碰撞临界，不一定发生碰撞；
- 非自由空间:栅格值为(0,127)，此时机器人处于障碍物附近，属于危险警戒区，进入此区域，将来可能会发生碰撞；
- 自由区域:栅格值为0，此处机器人可以自由通过；
- 未知区域:栅格值为255，还没探明是否有障碍物。
- 膨胀空间的设置可以参考非自由空间。
=== 2、设计方案

    - 配置环境  ：在Ubuntu20.04系统下安装ROS1（noetic）版本的环境，安装基本的依赖，导航功能包（movebase），建图功能包（gmapping），运动控制功能包（AMCL）等，使用VSCode进行代码编写
    - 机器人搭建：利用URDF搭建基础的仿真机器人，机器人必须包括激光雷达。
    - 仿真环境搭建：搭建Gazebo仿真环境，具体的素材可以在网上寻找
    - 控制节点配置：根据搭建的机器人结构配置对应的运动控制方式，例如两轮差速运动机器人。
    - SLAM建图：使用Gmapping功能包+键盘控制，进行SLAM建图，并利用地图服务节点保存地图。
    - 机器人路径规划与导航：使用上一步的地图进行移动机器人导航和路径规划实验。


== 四、实验步骤及操作方法

=== 4.1 安装虚拟机以及Ubantu
参考：https://blog.csdn.net/qq_45657288/article/details/116084337

=== 4.2 安装Ros以及VsCode
\

（1）打开虚拟机，打开终端

（2）安装ros：

执行命令：
```bash
        wget http://fishros.com/install -O fishros && bash fishros
```
#figure(
  grid(
  columns: (auto,auto),
  grid(  
    image("./image/4_2_2.png",width:100%,),
    image("./image/4_2_3.png",width: 100%,),
  ),
  grid(
    image("./image/4_2_4.png",width: 100%,),
    image("./image/425.png",width: 100%,),
    image("./image/426.png",width: 100%,),
    image("./image/427.png",width: 100%,),
  )
  )
)
(3)测试 ROS

ROS 内置了一些小程序，可以通过运行这些小程序以检测 ROS 环境是否可以正常运行
```
    首先启动三个命令行(ctrl + alt + T)

    命令行1键入:roscore

    命令行2键入:rosrun turtlesim turtlesim_node(此时会弹出图形化界面)

    命令行3键入:rosrun turtlesim turtle_teleop_key(在3中可以通过上下左右控制2中乌龟的运动)
```
最终结果如下所示:
#image("./image/428.png",width: 100%)

注意：光标必须聚焦在键盘控制窗口，否则无法控制乌龟运动。

（4）安装VScode

操作步骤同上，在输入命令后选择一键下载VsCode即可
=== 4.3 创建工作空间
打开终端依次执行以下命令，就能成功创建一个名叫catkin_ws的ros工作空间
```bash
      cd ~/    # 切换到主目录文件夹
      mkdir -p catkin_ws/src  # 创建工作空间，src用于保存功能包和代码
      cd ~/catkin_ws # 切换到工作空间目录下
      catkin_make # ros编译，编译后会生成一系列编译文件
```
=== 4.4 搭建仿真环境

创建机器人描述功能包，执行以下命令
```bash
  命令格式：catkin_create_pkg 功能包名 依赖库
  catkin_create_pkg mrobot_description urdf xacro
```
在功能包下新建文件夹用于保存不同类型的文件：
```
    config(rviz配置文件，机器人控制节点配置文件等) 
    launch（可运行节点）
    meshes（Gazebo仿真环境和机器人的材质）
    models（仿真模型）
    urdf（urdf格式的仿真模型）
    worlds（Gazebo环境）
```
==== 4.4.1 搭建机器人

在mrobot_description/models/wpb_home.model中编写了描述机器人结构信息的URDF格式的代码。
用于Gazebo仿真环境。它包括基础结构、主体、顶部、后部、头部和前部链接，以及激光雷达和Kinect v2传感器。每个链接都有视觉、碰撞和惯性属性，关节定义了它们之间的固定连接。

Gazebo插件用于模拟机器人的移动和传感器输出，IMU传感器提供角速度和加速度数据。整个模型通过精细的组件和传感器配置，模拟了一个具有高度复杂性和功能性的机器人。

进入urdf文件所属目录，调用:urdf_to_graphiz urdf文件，当前目录下会生成 pdf 文件
机器人的的节点描述如下：
#figure(
  grid(
  // columns: (1, 1),
  rows: (auto, auto),
  image(".\image\4\3机器人结构描述.png",format: "png",width: 100%) ,


)
)

打开终端 运行以下代码可以启动rviz查看机器人状态：
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch mrobot_description my_robot.launch
```
机器人可能出现显示错误，原因是没有odom里程计节点，把他换为任一节点即可
#figure(
  grid(
  // columns: (1, 1),
  columns: (auto, auto),
  image(".\image\4\4机器人加载错误.png",format: "png",width: 85%) ,
  image(".\image\4\5机器人正常显示.png",format: "png",width: 90%) ,

)
)
==== 4.4.2 搭建Gazebo仿真环境
\
在gazebo_test.launch中编写了加载Gazebo环境的xml代码，打开终端输入：
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch mrobot_description gazebo_test.launch
```
即可启动gazebo环境
#figure(
  grid(
  // columns: (1, 1),
  rows: (auto, auto),
  image(".\image\4\6gazebo仿真环境.png",format: "png",width: 85%) ,

)
)
把机器人加载进gazebo环境,只需要在gazebo_test.launch中加入:
```xml
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-file $(find mrobot_description)/models/wpb_home.model -urdf -x 0 -y 0 -model wpb_home" />
```
即可将机器人加载进gazebo仿真环境:
#figure(
  grid(
  // columns: (1, 1),
  rows: (auto, auto),
  image(".\image\4\7机器人加载进gazebo环境.png",format: "png",width: 85%) ,

)
)
==== 4.4.2 实现机器人运动控制

使用以下命令可实现键盘控制机器人运动
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
=== 4.5 SLAM建图

在终端运行:
```
roslaunch mrobot_description slam_map.launch 
```
即可开始执行slam建图服务
#figure(
  grid(
  // columns: (1, 1),
  columns: (auto, auto),
  image(".\image\4\8slam中rviz.png",format: "png",width: 85%) ,
  image(".\image\4\9slam中gazebo.png",format: "png",width: 85%) ,

)
)

再打开一个终端,执行以下命令,即可使用键盘控制机器人运动建图:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
#figure(
  grid(
  // columns: (1, 1),
  columns: (auto, auto),
  image(".\image\4\10键盘控制建图.png",format: "png",width: 85%) ,
  // image(".\image\4\9slam中gazebo.png",format: "png",width: 85%) ,
)
)
#figure(
  grid(
  // columns: (1, 1),
  columns: (auto, auto),
  image(".\image\4\11建图过程1.png",format: "png",width: 85%) ,
  image(".\image\4\12建图过程2.png",format: "png",width: 85%) ,
   image(".\image\4\13建图过程3.png",format: "png",width: 85%) ,
      image(".\image\4\14最终地图效果.png",format: "png",width: 85%) ,
)
)
编写地图保存save_map.launch文件
```xml
<launch>
    <arg name="filename" value="$(find mrobot_description)/map/my_map" />
    <node name="map_save" pkg="map_server" type="map_saver" args="-f $(arg filename)" />
</launch>
```
在终端执行
```bash
roslaunch mrobot_description save_map.launch 
```
即可将slam地图保存到本地
#figure(
  grid(
  // columns: (1, 1),
  columns: (auto, auto),
  image(".\image\4\15地图保存结果.png",format: "png",width: 85%) ,
  // image(".\image\4\9slam中gazebo.png",format: "png",width: 85%) ,
)
)
=== 4.6 机器人导航实现

执行:
```bash
roslaunch mrobot_description robot_navigation.launch 
```
开始机器人人导航
#figure(
  grid(
  // columns: (1, 1),
  columns: (auto, auto),
  image(".\image\4\16设置导航目标点.png",format: "png",width: 85%) ,
  // image(".\image\4\9slam中gazebo.png",format: "png",width: 85%) ,
)
)
#figure(
  grid(
  // columns: (1, 1),
  columns: (auto, auto),
  image(".\image\4\17开始导航.png",format: "png",width: 100%) ,
  image(".\image\4\18导航结束.png",format: "png",width: 100%) ,
)
)
在上图中机器人周围绿色的部分是局部地图路径规划的结果,红色线是全局路径规划的结果.
== 五、实验结果分析（代码上传至互联网并给出链接）

github链接如下
```url
https://github.com/zhouhandsome/Robot_intelligent_technology.git
```
\

实验结果证实了机器人仿真系统的成功搭建，SLAM技术在地图创建方面表现卓越，同时，机器人导航定位的准确性也达到了极高水准。这一系列成功的实验不仅展示了机器人在复杂环境中的导航能力，也为未来在实际应用中的进一步研究和开发奠定了坚实的基础。
== 六、结论与展望
\
=== 6.1 结论
\

在本次实验中，我成功地搭建了一个机器人仿真系统，并通过SLAM技术实现了高效准确的地图创建。机器人导航定位的精度极高，显示出了系统在复杂环境中的稳定性和可靠性。通过ROS的Navigation Stack，特别是move_base节点的集成，我们实现了精确的路径规划和动态避障。实验过程中，我深入探讨了成本地图的构建、传感器数据的融合以及参数优化等多个关键技术点，确保了机器人导航系统的高效运行。

=== 6.1 展望
\

尽管当前的实验成果令人满意，但我认识到机器人导航技术仍有进步空间。未来，我计划进一步优化算法，提高系统对动态环境变化的适应性和鲁棒性。此外，我也将探索多机器人协同工作的场景，以及如何通过机器学习技术提升机器人的自主决策能力。长远来看，我期望将这些研究成果转化为实际应用，为自动化物流、室内服务、灾难救援等领域提供技术支持，推动智能机器人技术的创新发展。