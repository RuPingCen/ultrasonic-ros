
# ultrasonic-ros

该模块是一个开源模块，并提供了配套的ROS节点，接收串口上传的数据并发布到指定的Topic上面。目前支持 HC-SR04 、HY-SRF05和US-015这几个型号超声波模块。主要功能如下:

- **1：** 同时读取12个超声波模块数据，通过串口打包上传，频率为20Hz。
- **2：** 提供配套ROS节点程序，可将测量数据发布到ROS Topic上。
- **3：** 开源ROS节点程序，并提供单片机端hex文件下载，使用其他单片机开发板也可以轻易使用。


板子为自制打样板，可以代为制作，手工焊接，仅收取成本和少量跑腿费（148/张），有需要可以在闲鱼搜索  
**2€2fEGczlp8YJ€ https://m.tb.cn/h.4lw1yij  我在闲鱼发布了【该模块是一个开源超声波读取模块，并提供了配套的ROS节點，接】**

更为详细的教程可以参考[博客-熊猫飞天](https://blog.csdn.net/crp997576280)

![fig](https://github.com/RuPingCen/blog/raw/master/ultrasonic-ros/fig/fig.jpg)

# 2 使用教程
## 2.1 接线说明
如下图所示，使用模块时需连接超声波模块
- P1-P12 为超声波连接1-12通道
- 电源输入电压为直流5-12V  DC-DC圆孔2.5mm
- 绿色灯为电源指示灯
- 红色灯闪烁频率大约为20Hz 表示正常测量过程

![fig2-1](https://github.com/RuPingCen/blog/raw/master/ultrasonic-ros/fig/fig2-1.jpg)

超声波接线说明，如下图： （ 红色——VCC 、 		黑色——GND 、		黄色——Trig 、 绿色——Echo）
  
![fig2-3](https://github.com/RuPingCen/blog/raw/master/ultrasonic-ros/fig/fig2-3.png)

超声波引脚对应图

![fig2-2](https://github.com/RuPingCen/blog/raw/master/ultrasonic-ros/fig/fig2-2.png)

SWD下载接口线序如下图所示（不需要更新程序则可以忽略该步骤）

![fig2-4](https://github.com/RuPingCen/blog/raw/master/ultrasonic-ros/fig/fig2-4.png)

## 2.2 上传协议
数据格式为： 帧头+数据长度+命令类型+数据+校验+帧尾。数据上传频率为20Hz，波特率使用115200。

![fig2-6](https://github.com/RuPingCen/blog/raw/master/ultrasonic-ros/fig/fig2-6.png)

- **帧头**：固定为两个字节： 0xAE, 0xEE
- **数据长度**：为所有字节的和（包含了帧头和帧尾）
- **命令类型**：固定为： 0xA1
- **数据位**：包含12个通道的超声波测量值，每个通道占用2个字节，共计24字节，采用高位在前方式发送，测量数据的单位为mm。 即为：[CH1_HSB  CH1_LSB … CH12_HSB  CH12_LSB]
- **校验位**：所有数据位的加和取低八位（包括帧头和帧尾）
- **帧尾**：固定为两个字节 0xEF,0xFE


# 3 ROS节点使用
## 3.1 下载与配置

 1. 安装依赖项
 ```
    sudo apt-get install ros-melodic-rosserial
```
 2. 下载编译代码
  ```
  cd catkin_ws/src

  git clone https://github.com/RuPingCen/ultrasonic-ros.git
  
  catkin_make
```
 
## 3.2 启动节点

启动节点以后可以看到终端打印输出的数据

```
  roslaunch ultrasonic_ros ultrasonic_ros.launch
```

![fig3-1](https://github.com/RuPingCen/blog/raw/master/ultrasonic-ros/fig/fig3-1.png)
     
使用 echo 命令打印查看超声波的数据

 ```
  rostopic echo /ultrasonic/data
```

![fig3-2](https://github.com/RuPingCen/blog/raw/master/ultrasonic-ros/fig/fig3-2.png)

使用 hz 命令查看话题发布的频率

 ```
  rostopic hz /ultrasonic/data 
```

![fig3-3](https://github.com/RuPingCen/blog/raw/master/ultrasonic-ros/fig/fig3-3.png)


## 3.3 常见问题-无串口权限

 启动节点时候提示没有串口设备或者没有启动权限，这时候需要修改串口权限。
 
**step1：** 检查是否识别到USB转串口驱动

![fig3-4](https://github.com/RuPingCen/blog/raw/master/ultrasonic-ros/fig/fig3-4.png)

查看设备的ID号    ls /dev/ttyUSB*

![fig3-4](https://github.com/RuPingCen/blog/raw/master/ultrasonic-ros/fig/fig3-5.png)

这里我们可以通过一个简单的方式来修改串口权限

```
sudo chmod 766 /dev/ttyUSB0
```

上述的方式每次拔插串口以后都需要执行修改权限的命令比较麻烦，可以使用我们的脚本文件把串口权限写入rule文件中。

**step2**: 用记事本打开“ultrasonic-ros/scripts/ultrasonic.rules” 目录下的文件，会看到如下内容

 ```
cd ultrasonic-ros/scripts

gedit ultrasonic.rules
```

 ```
# set the udev rule , make the device_port be fixed by rplidar
#
#KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0777", SYMLINK+="mick"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="mick"
```


**step3:** 这里的1a86和7523 就是上面图中我设备的ID号，这里修改成为你自己的ID号以后,把文件拷贝到/etc/udev/rules.d目录下就可以使用 /dev/ultrasonic  替代 /dev/ttyUSB0 来访问模块了

 ```
sudo cp ultrasonic.rules /etc/udev/rules.d/
```
