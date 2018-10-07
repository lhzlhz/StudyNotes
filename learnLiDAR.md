R# Velodyne VLP-16激光雷达

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
2. RangeImage(depth Image)的生成方法
    * cols自给，不要超过360/雷达分辨率。rows为雷达的线数。
    * 生成深度图像时，从点云生成的方法是：
      * 先确定每个col对应的角度，然后 atan2(point.x,point.y)求出方向角，再找哪个col最接近这个角，这便为这个point在深度图像中的col。
      * 该点的row为该点生成时的ring（推测为第几束激光射出去的编号） 
3. RangeImage的修复方法：
    * 遍历图像中，深度值小于0.01f的地方。
    * 若某点深度值小于0.01f，则搜寻该点上5行和下五行的，列数相同的点，这些点中，深度值相近的点会被采用，来计算该点的深度。举例来说，若点(i,j)深度值接近0，并且d(i+2,j) - d(i-3,j) < threshold，则把d(i+2,j)，d(i-3,j)的值存入sum变量中，同时计数器count += 2。该点的最终像素为sum/count。
4. 进行ground removal的三条假设：
    1. 激光雷达在机器人上几乎水平。
    2. 地面曲率不是很大。
    3. 至少在图像的最下面一行中的一些像素中检测到了地面。
5. 从深度图像创建AngleImage的方法。
    * 为何要创建AngleImage?该angle反映了所观测物体表面的斜率，这对去除地面很有效。
    * AngleImage的每个像素的计算公式由下给出
    $$\alpha = atan2(||\Delta z||, ||\Delta x||)$$
    $$\Delta z = |R_{r-1,c}sin\zeta_a - R_{r,c}sin\zeta_b|$$
    $$\Delta x = |R_{r-1,c}cos\zeta_a - R_{r,c}cos\zeta_b|$$  
      其中，$\zeta_a$ $\zeta_b$如图所示。    
      ![lidar](https://wp.me/aakuHy-1c)
6. 随后用Savitsky-Golay作为kernel调用cv::filter2d()来对AngleImage进行平滑。
   * Savitzky–Golay filter:通过对指定窗口大小的数据进行多项式拟合来尽可能保证信噪比的情况下，剔除outliers。![Savitzky-Golay filter](https://upload.wikimedia.org/wikipedia/commons/thumb/8/89/Lissage_sg3_anim.gif/600px-Lissage_sg3_anim.gif)
7. 然后运用广度优先搜索(BFS)来进行去除地面的操作。
   * BFS： 已知图G=(V,E)和一个源顶点s，算法自始至终一直通过已找到和未找到顶点之间的边界向外扩展，就是说，算法首先搜索和s距离为k的所有顶点，然后再去搜索和S距离为k+l的其他顶点。
   * 具体到该工程中是，把angle_image最后一行的每一列都加入队列中，对队列中的每个元素，进行搜寻它的四邻域，如果其angle与当前queue.front()小于thresh，则将其push到队列中。

           