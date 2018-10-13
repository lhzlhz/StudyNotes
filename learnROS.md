# ROS学习

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

## Client Libraries

> ROS client libraries允许nodes以不同的编程语言来通信。
> * rospy = python client library
> * roscpp =  C++ client library

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

## nodelet

nodelet的主要作用是在同一个机器同一个进程中运行多个算法，并且免去消息间通信时所消耗的数据拷贝时间。
* 应用场景：高吞吐量的数据流能够用多个notelet组成,并且被载入同一个process中以避免copying和网络堵塞。
* 技术：
  * 定义一个基类叫做nodelet::Nodelet用来动态载入(dynamic loading)。所有的nodelets可以继承该基类，并且也会因为使用pluginlib被动态载入。
  * 会自动提供命名空间，重映射的arguments和parameters，就像是一个node节点一样。
  * 会有一个在某个或者多个nodelet之间的nodelet_manager进程被载入。任何在其之间的通信都可以用零拷贝的boost shared指针来进行。 
* 基本语法
  * nodelet usage:
  * nodelet load pkg/Type manager - Launch a nodelet of type pkg/Type on manager manager
  * nodelet standalone pkg/Type   - Launch a nodelet of type pkg/Type in a standalone node
  * nodelet unload name manager   - Unload a nodelet a nodelet by name from manager
  nodelet manager               - Launch a nodelet manager node
* 编写一个nodelet的步骤：
    1. add the necessary #includes
    2. get rid of int main()
    3. subclass nodelet::Nodelet （基类nodelet::Nodelet，任何nodelet继承自它可以使用plugin的方式动态加载）
    4. move code from constructor to onInit() （实现onInit纯虚函数，用于初始化）
    5. add the PLUGINLIB_EXPORT_CLASS macro （加入宏，将子类声明为插件类，并编译为动态库）
    6. add <build_depend> and <run_depend> dependencies on nodelet in the package manifest.
    7. add the <nodelet> item in the <export> part of the package manifest
    8. create the .xml file to define the nodelet as a plugin
    9. make the necessary changes to CMakeLists.txt
    
    示例代码：
    ```cpp
    #include <pluginlib/class_list_macros.h>
    #include <nodelet/nodelet.h>
    #include <ros/ros.h>
    #include <std_msgs/Float64.h>
    #include <stdio.h>

    #include <math.h> //fabs

    namespace nodelet_tutorial_math  // The usage of the namespace is a good practice but not mandatory
    {

    class Plus : public nodelet::Nodelet
    {
    public:
    Plus(): value_(0)
    {}

    private:
    virtual void onInit() //当nodelet插件类被nodelet_manager加载时，nodelet插件类的onInit方法就会被调用，用于初始化插件类
    {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        private_nh.getParam("value", value_);
        pub = private_nh.advertise<std_msgs::Float64>("out", 10);
        sub = private_nh.subscribe("in", 10, &Plus::callback, this);
    }

    void callback(const std_msgs::Float64::ConstPtr& input)
    {
        std_msgs::Float64Ptr output(new std_msgs::Float64());
        output->data = input->data + value_;
        NODELET_DEBUG("Adding %f to get %f", value_, output->data);
        pub.publish(output);
    }

    ros::Publisher pub;
    ros::Subscriber sub;
    double value_;
    };

    PLUGINLIB_DECLARE_CLASS(nodelet_tutorial_math, Plus, nodelet_tutorial_math::Plus, nodelet::Nodelet);
    }
    ```
    注意为了允许类被动态加载，它必须被标记为导出类。这通过特殊宏PLUGINLIB_EXPORT_CLASS/PLUGINLIB_DECLARE_CLASS来完成，通常放在导出类的.cpp文件的末尾。 宏的参数分别为：pkg, class_name, class_type, base_class_type. 

    为了让pluginlib查询ROS系统上的所有可用插件，每个包必须显式指定它导出的插件。相应的package.xml中要加入下面内容：
    ```xml
    ...
    <build_depend>nodelet</build_depend>
    <run_depend>nodelet</run_depend>

    <export>
    <nodelet plugin="${prefix}/nodelet_math.xml" />
    </export>
    ...
    ```
    插件描述文件是一个XML文件，用于存储有关插件的所有重要信息。 它包含有关插件所在的库的信息，插件的名称，插件的类型等 。nodelet_math.xml如下：
    ```xml
    <library path="lib/libnodelet_math">
    <class name="nodelet_tutorial_math/Plus" type="nodelet_tutorial_math::Plus" base_class_type="nodelet::Nodelet">
        <description> 
        A node to add a value and republish.
        </description>
    </class>
    </library>
    ```

## Nodes

> 一个node在ROS package中其实就是一个可执行文件。ROS nodes除了可以订阅与发布Topic之外，也可以提供或者使用Service。

## parameter server

* Parameter server是一个共享的、多线程的字典，可以通过network api来获得。
* Parameter Server的数据类型
    * 32位整形
    * 布尔型
    * 字符串
    * 浮点型
    * iso8601 dates
    * lists
    * base64-encoded binary date 

## rosbag

* rosbag recode -e 正则表达式，可直接写topic名字 -O xxxname.bag
* rosbag play xxx.bag -l 循环播放

## rosbridge

rosbridge提供了json api以与非ros程序(WebSocket等)通信。

## roscore

> 当使用ROS时，应当首先运行roscore。

## rosed

rosed is part of the rosbash suite. It allows you to directly edit a file within a package by using the package name rather than having to type the entire path to the package.

usage:
$ rosed [package_name] [filename]

## ROS Graph ROS的图概念

* Nodes
    > 一个node通过ROS与其他node交流。
* Messages
    > ROS的数据类型，当订阅或者发布一个话题(topic)的时候使用。
* Topics
    > Nodes能够发布messages到topic，也能通过订阅一个topic来接收messages。
* Master
    > ROS的name service（推测翻译类似于主服务）。
* rosout
    > 在ROS中等同于stdout/stderr
* roscore
    > Master + rosout + parameter server(？)

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

## roslanch

usage:
$ roslaunch [package] [filename.launch]
* 当使用roslanch时，roscore未启动，则会默认启动roscore。

## ros logger levels

* Fatal
* Error
* Warn
* Info
* Debug

## rospack

* rospack depends1 [package] 会查找某个package的直接依赖。rospack depends [package]则会查找某个package的所有依赖。

## rosrun 语法

> $ rosrun [package_name] [node_name]

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

## Rviz

* rosrun rviz rviz -d 参数时，\`rospack find xxxx\` 不是用单引号而是用tab上面的“ \` ”符号。

## TF坐标变换

### TF工具

* static_transform_publisher:发布两个坐标系之间的静态坐标转换，这两个坐标系不发生相对位置变化。命令格式如下：
  ```shell
  $ static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
  $ static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms
  ```
  其参数第一部分为偏移参数，第二部分为旋转参数，旋转参数有两种表达形式：yaw/roll/pitch和四元数。
* view_frames:可视化调试工具，生成pdf文件，显示整个TF树的信息。常用命令如下：
  ```shell
  $ rosrun tf view_frames 
  $ evince frames.pdf # 查看pdf文件
  ```    
* tf_echo 工具的功能是查看指定坐标系之间的变换关系。命令的格式如下：
  > tf_echo \<source_frame\> \<target_frame\> 

## URDF

Unified Robot Description Format( 统一机器人描述格式 )。常用标签：
* \<link\> 用于描述机器人某个刚体部分的外观和物理属性。
```xml
    <link name="<link nmae>">
        <inertial>.......</inertial>
        <visual>.........</visual>
        <collision>......</collision>
    </link>
```
* \<joint\> 用于描述机器人关节的运动学和动力学属性。
```xml
    <joint name="<joint name>">
        <parent link="parent_link"/>
        <child link="child_link"/>
        <calibration ..../>
        <dynamics damping ..../><!--关节的物理属性-->
        <limit effor ..../>
        ....
    </join>
```  

## 坑

* ros kinetic + ubuntu16.04 + gcc 5.0+ 会导致链接问题：undefined reference to `ros::console::initialize()……'
* 在launch file里添加命令行参数时，会多添加“\__name:nodename”，“\__log:xxxx.log”两个命令行参数，会导致cmd paser失败。