# Velodyne VLP-16激光雷达

## 数据类型

1. 球坐标系：
 以坐标原点为参考点，由方位角（azimuth $\phi$、仰角（elevation $\omega$）和距离（radius $\gamma$）构成。原点与点 P 之间的径向距离 r ，原点到点 P 的连线与正 z-轴之间的天顶角$\theta$以及原点到点 P 的连线，在 xy-平面的投影线，与正 x-轴之间的方位角$\phi$。
![球坐标](https://gss2.bdstatic.com/9fo3dSag_xI4khGkpoWK1HF6hhy/baike/c0%3Dbaike92%2C5%2C5%2C92%2C30/sign=6c2bb489be003af359b7d4325443ad39/3ac79f3df8dcd100e059aca17a8b4710b8122f6c.jpg)

2. 传感器返回在球坐标系中相对于其自身的距离坐标。传感器数据的原点比其自身高出37.7mm。

3. 球坐标向笛卡尔坐标的转换：

    $$X = R*cos(\omega)*sin(\alpha)$$
    $$Y = R*cos(\omega)*cos(\alpha)$$
    $$Z = R*sin(\omega)$$
    其中$\omega$是图中$\theta$的余角，$\theta$是$\phi$的余角。

4. Savitzky–Golay filter:通过对指定窗口大小的数据进行多项式拟合来尽可能保证信噪比的情况下，剔除outliers。![Savitzky-Golay filter](https://upload.wikimedia.org/wikipedia/commons/thumb/8/89/Lissage_sg3_anim.gif/600px-Lissage_sg3_anim.gif)

## 激光返回模式(laser return modes)：

1. Single Return Mode:只取同一束激光的一种返回值，比如说取光强最强的(Strongest)，或者最后返回的(Last)
2. Dual Return Mode：同时获得Strongest和last

## C语言知识：

1. typeid:
  
    1. 当typeid操作符的操作数是不带有虚函数的类类型时，typeid操作符会指出操作数的类型，而不是底层对象的类型。
    2. 如果typeid操作符的操作数是至少包含一个虚拟函数的类类型时，并且该表达式是一个基类的引用，则typeid操作符指出底层对象的派生类类型。

## deep_clustering

1. A-->B 表示 A是B的基类，则有：
    * identifiable-->AbstractClient, identifiable-->AbstractSender。identifiable类的作用主要是给继承该类的子类提供唯一的id，以及显示其类名的方法（见identifiable::guess_class_name()）。
    * AbstractClient与AbstractSender组成了一个发布者/订阅者模型。AbstractSender通过调用其ShareDataWithAllClients()方法来调用所有client的OnNewObjectReceived()方法来处理数据。
    * AbstractSender\<cloud\>-->CloudOdomRosSubscriber
    * AbstractClient,AbstractSender-->DepthGroundRemover。DepthGroundRemover作为CloudOdomRosSubscriber的订阅者，接收点云信息；同时它又作为image_based_cluster的发布者，将其去除地面后的点云信息发布给后者。
    * CloudProjection类负责把点云投影成深度图像。