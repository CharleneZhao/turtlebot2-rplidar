# turtlebot2-rplidar

使用turtlebot包和rplidar_ros包，更改了其中一些配置文件

实现建图，在已知地图上的动态导航避障

gd_bus_bot.py实现控制turtlebot去到A位置取货物再运输到B位置的功能。

基于turtlebot包+rplidar，实测机器人定位比rbx1稳定很多，rplidar参数不同？

使用方法：

1.bringup机器人:
roslaunch turtlebot_bringup minimal.launch

2.rplidar建图：
roslaunch turtlebot_navigation rplidar_gmapping_demo.launch

3.rplidar自主行驶：
roslaunch turtlrbot_navigation rplidar_amcl_demo.launch 

rviz: roslaunch turtlebot_rviz_launchers view_navigation.launch --screen

存地图：rosrun map_server map_saver -f my_map

运输机器人代码：
/catkin_ws/src/gd_bus_bot.py
