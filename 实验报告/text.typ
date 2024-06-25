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

==== 1.3 Gazebo和rviz机器人仿真平台介绍
==== 1.4 Slam步骤
==== 1.5 路径规划实现（局部地图，全局地图，碰撞算法，规划算法）
=== 2、设计方案

    
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
（3）安装VScode

操作步骤同上，在输入命令后选择一键下载VsCode即可
=== 4.3 创建工作空间

  （1）打开终端

```bash
      cd ~/    # 切换到主目录文件夹
      mkdir -p catkin_ws/src  # 创建工作空间，src用于保存功能包和代码
      cd catkin_ws # 切换到工作空间目录下
      catkin_make # ros编译，编译后会生成一系列编译文件
```

=== 4.4 搭建仿真环境
=== 4.5 SLAM建图
=== 4.6 机器人导航实现
== 五、实验结果分析（代码上传至互联网并给出链接）

== 六、结论与展望
