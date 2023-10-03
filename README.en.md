# introduction
本功能包为拓渝科技有限公司的智能网联传感器教学软件，
可用于雷达、相机、组合惯导传感器教学；
# install 
1. 安装ros
2. 下载功能包
git clone git@github.com:ZhawedHoo1998:tuoyu_sensorteaching.git
3. 安装相关依赖
sudo apt install ros-melodic-jsk-rviz-plugins
sudo apt install ros-melodic-grid-map
sudo apt install ros-melodic-velodyne
sudo apt install ros-melodic-costmap-converter
4. 安装socket-can
5. 编译运行
catkin_make

