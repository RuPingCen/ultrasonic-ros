
# ultrasonic-ros

多超声波模块读取ROS节点，该模块支持同时读取12个通道的超声波模块数据，目前支持（HC-SR04）。该节点为模块配套的ROS节点，用于接收模块上传的数据，并发布到ROS Topic上
 ## 1.1 下载与配置

 1. 安装依赖项
 
    sudo apt-get install ros-melodic-rosserial
    
 2. cd catkin_ws/src
 
 3. git clone https://github.com/RuPingCen/ultrasonic-ros.git

 4. catkin_make
 
 ## 1.2 启动节点
 
     roslaunch ultrasonic_ros ultrasonic_ros.launch
     
 
