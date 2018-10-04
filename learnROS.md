# ROS学习

## ROS的图概念

* Nodes
    > 一个node通过ROS与其他node交流。
* Messages
    > Ros的数据类型，当订阅或者发布一个话题(topic)的时候使用。
* Topics
    > Nodes能够发布messages到topic，也能通过订阅一个topic来接收messages。
* Master
    > ROS的name service（推测翻译类似于主服务）。
* rosout
    > 在ROS中等同于stdout/stderr
* roscore
    > Master + rosout + parameter server(？)

## Nodes

> 一个node在ROS package中其实就是一个可执行文件。ROS nodes除了可以订阅与发布Topic之外，也可以提供或者使用Service。

## 客户端库(Client Libraries)

> ROS client libraries允许nodes以不同的编程语言来通信。
> * rospy = python client library
> * roscpp =  C++ client library

## roscore

> 当使用ROS时，应当首先运行roscore。

## rosrun 语法

> $ rosrun [package_name] [node_name]

## catkin

使用catkin_make的一般流程：

``` bash
$ cd ~/catkin_ws/src/beginner_tutorials/src
# Add/Edit source files
$ cd ~/catkin_ws/src/beginner_tutorials
# Update CMakeFiles.txt to reflect any changes to your sources
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
```

执行完catkin_make之后，生成的文件会被放在devel space(~/catkin_Ws/devel)

## rostopic

*turtle_teleop_key*发布按键事件到一个topic上，*turtlesim*订阅该topic来得到按键事件。

subscriber与publisher之间是通过ROS Messages来通信的。Publisher和subscriber必须发布和接收同种类型的数据。

rostopic语法

|语法|解释|
|---|----|
|rostopic bw    | display bandwidth used by topic|
|rostopic echo  | print messages to screen|
|rostopic hz    |display publishing rate of topic|
|rostopic list  |print information about active topics|
|rostopic pub    |publish data to topic|
|rostopic type   |print topic type|

## rosserive

rosservice是另一种节点间可以相互通信的方法。
|语法|解释|
|-----|----
|rosservice list  |       print information about active services
|rosservice call   |      call the service with the provided args
|rosservice type    |     print service type
|rosservice find     |    find services by service type
|rosservice uri       |   print service ROSRPC uri
* paramter server

## ros logger levels

* Fatal
* Error
* Warn
* Info
* Debug

## roslanch

usage:
$ roslaunch [package] [filename.launch]
* 当使用roslanch时，roscore未启动，则会默认启动roscore。

## rosed

rosed is part of the rosbash suite. It allows you to directly edit a file within a package by using the package name rather than having to type the entire path to the package.

usage:
$ rosed [package_name] [filename]

## ROS msg 和 srv

* msg: msg文件是简单的text文件，里面包含了字段类型， 变量名称。可用的字段类型为：
  * int8, int16, int32, int64 (plus uint*)
  * float32, float64
  * string
  * time, duration
  * other msg files
  * variable-length array[] and fixed-length array[C]

* 另外还有中特殊格式:Header，包含时间戳以及其他信息。
* srv文件和msg文件类似，不同的是它由两部分组成：request和response。这两部分被用‘---’分割。

## cpp中各种ROS函数的作用

1. ros::init(int argc, char **argv, char *nodeName)：  
    >使用任何ROS函数之前必须先使用ros::int。
2. ros::NodeHandle:
    >NodeHandle是ROS通信的主要方式。
3. ros::NodeHanle::advertise():
    >第一个参数是要发布的topic名字，第二个参数是需要缓冲的数据数量，当消息产生比发送快很多时，该参数决定了多少消息在被丢掉之前会被缓存。返回值类型为ros::Publisher。调用ros::Publisher::publish()可以发布消息。
4. ros::Publisher::publish()
    >唯一的参数是message object，该类型应该和davertis<>的模板类型相同。
5. ros::rate
    >ros::rate 决定了，当你调用ros::rate::sleep()函数时，休眠的长度，保证每次循环的周期。
6. ros::ok()
    >在以下情况下会返回false：
    >   * a SIGINT is received (Ctrl-C)
    >   * we have been kicked off the network by another node with the same name
    >   * ros::shutdown() has been called by another part of the application.
    >   * all ros::NodeHandles have been destroyed
7. ros::spinOnce()
    > 执行一次callback函数。
8. ros::NodeHandle::subscribe():
    > 三个参数，第三个参数为callback函数的指针，前两个参数和advertise相同。
9. ros::spin():
    > 进入循环，如果有发布消息则执行callbakc函数，没有则进入休眠状态。

##　ROS中的bag

## rospack

* rospack depends1 [package] 会查找某个package的直接依赖。rospack depends [package]则会查找某个package的所有依赖。

## rosbridge

rosbridge提供了json api以与非ros程序(WebSocket等)通信。

## message_filters

message_filters是消息滤波器。

* time synchronizer：时间同步器，同时接收不同源的消息，但只输出时间戳相同的消息。
* 滤波器模式(Filter Pattern):所有消息滤波器对于输入输出的链接都有相同的模式。输入连接到滤波器的构造函数或者是其connectInput()方法上。输出连接到其registerCallback()方法上。
* Subscriber:订阅者滤波器。是一个ROS subsription的包装。Subscriber滤波器不能够连接另一个滤波器的输出。它的输入为一个ROS topic。下面两个写法是等效的。

    ``` cpp
    message_filters::Subscriber<std_msgs::UInt32> sub(nh, "my_topic", 1);
    sub.registerCallback(myCallback);
    ```
    等效于
    ``` cpp
    ros::Subscriber sub = nh.subscribe("my_topic", 1, myCallback);
    ```
* Policy-Based Synchronizer:
  * Synchronizer滤波器输入为至多九个分开的滤波器，每个都是void callback(const boost::shared_ptr\<M const\>&)的形式。
  * 对于输入的消息类型M0..M8，输出是void callback(const boost::shared_ptr\<M0 const\>&, ..., const boost::shared_ptr\<M8 const\>&).
  * 其时间同步有两个方法:
    1. message_filters::sync_policies::ExactTime 需要消息有完全一致的时间戳去匹配。回调函数只有在所有channels接收到相同时间戳的消息才会被调用。
    2. message_filters::sync_policies::ApproximateTime：算法比较复杂，个人理解如下。
      * 首先，将该算法的输出称为集合(set)，即消息类型不同，但时间戳相近的message的集合。
      * 当某个集合$S$被发布后，所有topic中，比$S$中相应的topic的message早的message都会被丢弃。
      * 当每个topic都至少拥有一个message后，令时间戳最迟的那一条消息为主元(pivot)。
      * 然后查找其他topic中所有比主元早的消息，看看他后面一条消息时间是否是比主元迟，是的话则该消息成为将要发布的set$L$的一分子。
         
        ![ApproximateTime.png](http://wiki.ros.org/message_filters/ApproximateTime?action=AttachFile&do=get&target=ApproximateTimeExample.png)

## 坑

* ros kinetic + ubuntu16.04 + gcc 5.0+ 会导致链接问题：undefined reference to `ros::console::initialize()……'
* 在launch file里添加命令行参数时，会多添加“\__name:nodename”，“\__log:xxxx.log”两个命令行参数，会导致cmd paser失败。