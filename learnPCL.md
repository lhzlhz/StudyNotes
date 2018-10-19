# PCL点云库学习

## Organized point cloud Vs unorganized point cloud

  * Organized pcl 一般是从双目相机或者tof相机中获得的。
  * 相较于unorganized point cloud，优势在于用最近邻算法时，由于知道相邻点的关系，所以更加高效。

## PCL Walkthrough

PCL的各个模块的简单介绍。
1. Filters(过滤器)
   * 一个例子是通过一系列统计学上的算法，计算每个点到其他点相邻点的距离，假设所有点的分布满足高斯分布，去除一定interval之外的outlier。
2. Features(特征)
   * features 库中的算法主要来估算点云数据的三维特征。一个经常用到的特征是检测underlying surface是否是有曲率的。
3. Keypoints
   * Keypoints库中包含了两种点云库特征点识别的算法。Keypoints是点云中稳定的、有代表性的点，并且可以用定义完备的检测标准来识别。
4. Registration
   * Rsgistration是一种技术，用来把多个数据集结合到一个统一的模型中。关键思想是识别出数据集之间的相关点并且找到一个变换，使得相关点的distance（或者叫alignment error)最小。
5. Kd-tree
   * Kd-tree库提供了kd-tree数据结构，使用FLANN，并且使得能够运用快速的nearest neighbor searches。
6. Octree
   * Octree库提供创建有层次的树状数据结构的高效方法。每一棵Octee节点有八个子节点或者没有子结点。根节点描述了一个方形box边界来包裹住所有的点。每一层树level中，该空间被一分为二来增加体素分辨率（voxel resolution）。
7. Segmentation
   * Segmentation库包含了将点云分割成特定集合的算法。
8. Sample Consensus
   * Sample Consensus库包含了随机抽样一致算法例如(RANSAC)等。
9.  Surface
   * Surface库中的算法将原始表面从三维传感器中重建出来。
10. Range Image
   * range_image库中包含了两个类，用来显示深度图像，或者处理深度图像。
11. I/O
   * IO库中包含读写PCD文件的函数。
12. Visualization
   * visualization库用来可视化算法的结果。
13. Common
   * common库中包含了一般的数据结构和被大部分PCL库所调用的算法。
14. Search
   * search库中提供了对于不同种数据的最近邻算法，包括：
     * KdTree
     * Octree
     * brute force
     * specialized search for organized datesets.
15. Binaries
   * 提供了一些PCL中的common tools的快速引用。 

## Basic Structure

1. pcl::PointCloud
   * ::is_dense
   * TODO: 被学长粗暴地打断
  
## Cluster

1. EuclideanClusterExtraction 欧式距离分类

## How to use a KdTree to search

* 二分查找树--左子树上所有结点的值均小于根节点的值，右子树上所有结点的值均大于它的根结点的值。
* FLANN--Fast Library for Approximate Nearest Neighbors，是一个执行快速近似最近邻搜索的库。
* k-d tree--k维树结构。k-d树为二分查找树。因为处理点云，所以这里的k=3。
  * 其所有叶节点是k维的点。所有非叶节点可以被想象成超平面来将K维空间分成两部分，称之为半空间(half-spaces)。在根节点处所有的点会被分成两部分，根据他们第一维的值。每层结构都会使用别的维度来分割点。
  ```
  function kdtree (list of points pointList, int depth)
  {
      // Select axis based on depth so that axis cycles through all valid values
      var int axis := depth mod k;
          
      // Sort point list and choose median as pivot element
      select median by axis from pointList;
          
      // Create node and construct subtree
      node.location := median;
      node.leftChild := kdtree(points in pointList before median, depth+1);
      node.rightChild := kdtree(points in pointList after median, depth+1);
      return node;
  }
  ```
  ![一个wikipedia的例子](https://upload.wikimedia.org/wikipedia/commons/thumb/b/bf/Kdtree_2d.svg/555px-Kdtree_2d.svg.png)
  
  k-d tree decomposition for the point set(2,3), (5,4), (9,6), (4,7), (8,1), (7,2)
  ![](https://upload.wikimedia.org/wikipedia/commons/thumb/2/25/Tree_0001.svg/555px-Tree_0001.svg.png)
  The resulting k-d tree.
* 最近邻搜索在Kdtree上的应用。
    * 搜索步骤如下：
      1. 从根结点开始往下数。如果目标点的当前分割的维度的值，小于根结点的值，则将左结点设置为currunt best。否则设置右结点为current best。
      2. 但同时它又设置了一个超球体，以目标点为球心，半径为current best和目标点的距离，假如该球体与分割的超平面相交的话，有可能在右边的节点处有比current best更近的点，所以从右边的点处以同样的规则往下数。
      3. 如果没有相交，那么该算法继续走，并且整个右侧的结点都不用考虑了。
  
