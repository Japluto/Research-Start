# 了解利用VICON动捕系统搭建的实物平台框架

> Vicon视频教程（百度网盘）：https://pan.baidu.com/s/1LtNESSpV9HC7ecRJzdYiww?pwd=ixqt

下面以岳江源同学的本科毕设框架为例，进行介绍

![](attachments/Pasted%20image%2020221110214404.png)

Vicon动捕系统，顾名思义，主要作用是提供目标物体的 **位置** 和 **姿态** 信息。

如上图所示，通过多个红外摄像头作为传感器，识别预跟踪物体上的 marker 球，并且通过交换机与 Vicon 电脑通信，Vicon 电脑上有相应的官方软件 **ViconTracker** ，软件解算相应的信息，并且通过预先存储好的物体刚体模型，进行跟踪，而后通过接入局域网，向局域网中广播这些信息，最后在我们自己使用的电脑上进行相应的配置，便可接收到相应信息。


# VICON 电脑的配置

可以查看上面视频教程

## 1- 电脑的IP设置（已完成）

将交换机接出一根网线到电脑下面的千兆网口，然后在摄像头和交换机运行的情况下进入网络共享中心，设置以太网二，即连接交换机的网。

点击进入后，点击属性。然后点击配置，点击高级。设置以下参数。

`Jumbo Packet` 调至9014 Bytes（没有就最大）。`Receive Buffers` 2048或最高。`Transmit Buffers` 2048或最高。

回到属性界面，设置 Internet 协议版本4（TCP/IPV4）属性。设置IP 192.168.10.1，子网掩码为255.255.255.0。

![](attachments/Pasted%20image%2020221110214416.png)

## 2- 软件设置

1. 确保所有硬件连接完毕，交换机电源开启，电脑开启，加密狗(指蓝色外壳，挂有VICON小牌的形如U盘的物品，是提供软件使用权限的)插上电脑，且红灯亮, 交换机有一根网线连接到电脑下面的千兆网口，且电脑IP设置完毕。
2. 双击桌面软件，ViconTracker

    ![Alt text](attachments/image1.png)

3. 等待镜头识别，相同数量的摄像头前面的图表变为绿色

    ![Alt text](attachments/image2.png)

## 3- 调节镜头位置（已完成）

1. 放置 marker 球在采集区域边缘，软件内中间视图框内左上角，点击3D perspective，选择camera，然后在左侧resources栏目选择摄像头。
2. 一人在电脑前查看，一人调节镜头上下左右，直至摄像头视野内完整的出现边缘区域的点。

## 4-系统标定

> 已经完成，但是根据场景灯光变化以及不可控因素，可能需要多次进行，具体根据实际效果决定

1. 设置镜头参数，放置一定数量的 marker球 在环境中，选择摄像头视野，选中摄像头，调节 threshold 和 minimum circularity ratio,直至视野中的点稳定不闪烁，杂点尽可能少。

    ![](attachments/Pasted%20image%2020230619152016.png)

2. 每个摄像头都设置完毕后，移除场地内的 marker球 ，点击 calibrate 分页，点击 Create camera masks 下的 start ，等待几秒，直至摄像头视野内除了黑色只剩下蓝色方块。

    ![](attachments/Pasted%20image%2020230619152106.png)

3. 点击 calibrate 下的start，然后让另一人拿标定架在场地内挥舞标定架。点击这里，注意查看，左下feedback内的内容，当采集数量足够后，标定将自动停止，进行计算，同时feedback内的进度条逐渐跑动，直至跑满，检查world error都低于0.5则满足要求，如果不满足，重复1-3内容。

## 5-建立刚体

1. 将所测物体贴上 marker球（至少3个），置于采集空间内，点击 OBJECT 分页，按住 alt按键 拖动进行复选，框住所有 marker ，然后在下方，create object 处，输入刚体名称，点击 create 按键，建立刚体。

2. 如果想要建立第二个刚体，先点击下方的track按键，让其处于灰色的状态，然后重复1来建立第二个刚体。

3. 刚体在被建立时，其6自由度的旋转角度 Rx,Ry,Rz ,三个角度将被强制设置为0，其刚体的位置自由度Tx,Ty,Tz,为刚体上所有 marker球 的几何算术中心。

4. 如果想要修改初始角度，比如设置为Rx,Ry,Rz为0,90,0，建立完刚体后，点击 Object 右侧的暂停符号（或者点击键盘的空格），选择所需的刚体，在下方 property 内点击 show advanced ， 然后输入所需角度，之后再取消暂停即可。

## 6-后续

如果只是实时采集，实时控制，那么进行到此步，就已经基本配置完成了。**若是需要非实时采集，后期分析，则可继续查看下一步**。

## 7-采集，查看，输出

1. 采集数据： 点击 recording 分页，在 trial name 内输入你所需要的名字，回车后，点击 start 按键进行采集，采集足够多数据后，点击 stop 。

2. 查看数据：点击 playback 内 load trial ，选择你所存的文件，点击确定。（默认存储路径为C:\Users\Public\Documents\Vicon\Tracker3.x\CapturedTrials）

3. 输出数据：点击 export CSV 下的 expor 按键即可在指定路径输出数据。（注意输出数据中仅包含刚体数据，没有marke坐标数据，同时Rx,Ry,Rz的数据不是用的角度，而是弧度）




# 个人电脑的配置

> 个人电脑与Vicon电脑的通信主要有两种方式，一种是直接将个人电脑用网线与交换机相连（不推荐），另一种是连接上Vicon电脑相同的局域网。两种方式不同的地方在于最后设置的IP地址，前一种需要设置Vicon电脑与交换机相连的IP地址，后一种则是连接的局域网的IP地址。具体信息可以在Vicon电脑上查看。

## ubuntu系统下与ROS的配置

下载编译与Vicon动捕系统相应的ROS工具包，可以直接在github上搜索，这里列举几个常用的，根据使用的包不同，能够得到的信息也不同，信息最丰富的包可以获取位置，姿态，速度，角速度，以及相应的协方差矩阵信息等。

1.[Vicon_bridge](https://github.com/ethz-asl/vicon_bridge)

待后续补档......


此外，在一些项目中已经内置了Vicon的功能包。

在配置方面，按照自己下载的 ROS 包说明进行参数修改。但都需要配置IP地址。

以CrazySwarm中的配置举例，将VICON PC 连接至局域网中，然后用路由器连接相同的局域网后，使用个人笔记本电脑连接路由器对应的无线网即可。而后再设置CrazySwarm中的``ros_ws/src/crazyswarm/launch/hover_swarm.launch``文件，在以下文段处进行设置：

```js
\#ros_ws/src/crazyswarm/launch/hover_swarm.launch
motion_capture_type: "vicon"
motion_capture_host_name: "viconPC" # hostname or IP address
```

将`motion_capture_host_name`的值设置为连接的无线网的IP地址即可。
