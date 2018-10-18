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