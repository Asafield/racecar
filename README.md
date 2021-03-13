# 技术报告

## 项目背景和意义

- 概述项目背景和意义
- 项目所要解决的问题
- 创新点
## 项目方案设计
- 方案的设计思路
- 方案的对比
- 采用方案的特点
## 项目算法功能
- 算法和模型以及所实现的功能介绍
- 算法实现思路或算法优化思路
## 项目实施过程
- 项目方案的具体实现过程及步骤

  ​		**通过对比并决定了最终的方案设计之后，就进入到了方案的实现环节。具体的实施过程包括对数据的采集和处理、各种分析工具的使用和对比，同时，也遇到了一些问题，通过多次的尝试和不断的测试，在解决一个个问题的基础之上完成了整个方案的优化和实现。**

  - 数据的采集、处理

    **数据采集-传感器**

    - 激光雷达

      ​		对于运动中的机器人来说，对周围障碍物的位置的识别，空间的轮廓信息的获取是非常重要的，同时，这对于机器人创建地图一个导航定位来说，也是不可或缺的。而通过激光雷达获取这些信息自然是不二之选，从而测量机器人和周围环境的距离，并通过LaserScan的话题类型发布数据。

      ![激光雷达图](/home/asafield/.config/Typora/typora-user-images/image-20210313133146781.png)

    

    - 里程计

      ​		里程计通过传感器收集数据来估计机器人在里程计坐标系中的随着时间变化的位置，常见的里程计有激光里程计，轮式里程计等等。但是这些里程计都会随着时间产生累计误差。需要ekf(扩展卡尔曼滤波)以及 amcl(自适应的蒙特卡洛定位法)定位并纠正误差。而本次仿真使用的是由gazebo插件提供的里程计，基于直接读取在gazebo仿真世界中的坐标进行定位，理论上来说不存在误差，因此用作机器人的定位非常精准，也应此放弃使用ekf融合数据，但保留amcl用以发布map坐标系到odom坐标系的tf转换。

      ![里程计数据](/home/asafield/.config/Typora/typora-user-images/image-20210313135110711.png)

      ​		

    - tf变换

      ​		在仿真世界中，机器人各个组件之间、机器人本身与外界环境之间的相对位置关系是通过坐标变换实现的。在本次方案设计中，涉及到坐标变换的，主要有机器人本身的xacro模型文件中的坐标系，以及odom里程计坐标系和map世界坐标系。其中，机器人各个组件的坐标变换由robot_state_publisher节点发布；odom坐标系由gazebo插件发布;map坐标系由amcl发布。这样，整个仿真的tf变换关系就搭建完成了：

      ![tf变换](/home/asafield/.config/Typora/typora-user-images/image-20210313143955933.png)		

      ![示例](/home/asafield/.config/Typora/typora-user-images/image-20210313144425242.png)

      

  - 分析工具的使用

    - rqt工具
      - rqt_graph（数据绘图工具）
        		     rqt_graph是可以将当前ROS系统中的计算图通过图形化的方式展示出来的工具。在使用时，可以直观的展示所有启动的节点和话题，并将各节点之间消息流清晰的展示出来。
      
        ![rqt_graph](/home/asafield/.config/Typora/typora-user-images/image-20210313152133793.png)
    
      - rqt_plot(计算图可视化工具)
                   rqt_plot是一个非常方便的曲线绘制工具，可以将一些消息数据以随时间变化的曲线绘制出来。是一种非常方便的数据可视化工具，本次设计中常在速度调试时使用改工具。              
      ![rqt_plot](/home/asafield/.config/Typora/typora-user-images/image-20210313150645850.png)
             
       - rqt_reconfigure(参数动态配置工具)
      
            rqt_reconfigure是可以在不重新启动仿真的情况下情况下，进行参数的动态配置的工具。在本次设计中，对于数量较多的参数，类似move_base功能包的参数，尤其是teb_local_planner相关参数，使用此工具进行配置十分便捷。
            
            ![image-20210313153716320](/home/asafield/.config/Typora/typora-user-images/image-20210313153716320.png)		
            
        - rqt_tf_tree（tf变换查看工具）
      
            rqt_tf_tree可以通过命令查看当前仿真中建立的tf坐标变换，直观而简洁地展示整个仿真世界中的坐标系之间的关系。
      
            ![image-20210313154055887](/home/asafield/.config/Typora/typora-user-images/image-20210313154055887.png)

  - 建图

    建图采用的算法是cartographer算法，是基于图网络的优化方法，对于内存和计算资源的要求更低。主要分为local slam和global slam两部分，在local slam中，依靠传感器（里程计和imu）数据进行轨迹推算，得出机器人的位姿估计。并通过雷达数据进行滤波和叠加后形成一系列的子图（submap）,而在global slam中，通过回环检测和后端优化得到一张完整的地图。

    ![cartographer](https://img-blog.csdnimg.cn/20191230155220786.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjE1NjA5Nw==,size_16,color_FFFFFF,t_70)

    + cartographer配置运行

      cartographer的配置主要是通过lua文件和launch文件完成的，其中最主要就是配置好lua中的参数。

      <img src="/home/asafield/.config/Typora/typora-user-images/image-20210313160900905.png" alt="lua" style="zoom:70%;" />

      cartographer建图过程

      ![cartographer建图过程](/home/asafield/.config/Typora/typora-user-images/image-20210313162402351.png)


  - 导航

    本次设计导航依靠的主要功能包是move_base，其中全局路径规划和实施路径规划采用global planner和local planer，运动控制采用teb local planner的参数调节和自己编写的控制算法结合的方式。

    - 路径规划

      路径的算法采用Dijkstra算法，作为一种广度优先的算法，在这种地图不大，不需要太多的计算资源的仿真中无疑更有优势。在本次设计中，global planner 规划出一条全局路径，而local planner 规划的局部路径在考虑实时避障的情况下尽量逼近全局路径。

      ![路径规划](/home/asafield/.config/Typora/typora-user-images/image-20210313173044341.png)

     - 控制算法

       机器人控制方面我们主要从三方面进行了考虑：

       1. teb local planner：作为路径规划算法，teb也提供了大量的参数对机器人的速度、避障等方面进行调节，因此，要想控制好机器人的控制，是离不开对这些参数的优化的。

       2. skid_steer_drive_controller：本次线上比赛使用的机器人是四轮差速驱动的，而gazebo控制插件也并非常用的gazebo_ros_control，而是一个四轮驱动插件skid_steer_drive_controller，因此，要想控制好机器人，尤其是控制转弯，了解该插件控制四轮差速机器人的原理就变得非常重要了。

          ![四轮驱动插件部分源码](/home/asafield/.config/Typora/typora-user-images/image-20210313170332441.png)

       3. 自编写控制算法：为了提升速度，需要对控制算法进行优化，其中最直接的就是直道加速，弯道减速了。而实现这种效果的方法很多，我们最后实现的是：通过选取机器人移动时前方一段路程的三个点作外接圆，其半径作为曲率半径，当前方一段路程的最小曲率半径达到某个值时，机器人就减速。由此实现在全图灵活的控制速度。

          ![curvature](/home/asafield/.config/Typora/typora-user-images/image-20210313172910851.png)

  - 所遇到的困难及解决的方法

    - 困难：

    	1. 使用cartographer建图时，偶尔会出现地图漂移重合的问题

    	2. 运动控制时，机器人转弯速度不够快，且由于小车惯性较大，容易出现转弯和最后结束的时候刹不住车的问题。

     - 解决方法：
        1.在cartoprapher的lua文件中可以选择使用里程计和imu，从而增加定位精度，同时在建图过程中使机器人低速移动以得到准确度更高的地图。
        2.编写直到加速，弯道减速的控制程序，以达到灵活转弯的效果，在最后停车时，通过终点前减速来平稳停车。 
## 项目数据分析
- 对测试数据的结果进行分析对比
- 论证方案的可行性和有效性
  - 数据来源
  - 数据处理
  - 实验环境配置
  - 测试过程
  - 分析与结论
## 项目作品总结
- 自我评价和总结
  - 项目方案的创新点
  - 项目所实现的功能
  - 测试的数据结果
- 展望
  -作品的进一步提升和应用拓展
## 参考文献及附录
- 参考文献
- 附录
  1. 参考内容(可选)
      - 公式的推演
      - 编写的算法语言程序设计
      - 图纸、数据表格
  2. 程序源代码
  3. 研究论文(可选)

