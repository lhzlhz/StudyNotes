# C++学习

## boost::shared_ptr<>
std::shared_ptr 是通过指针保持对象共享所有权的智能指针。多个 shared_ptr 对象可占有同一对象。下列情况之一出现时销毁对象并解分配其内存：

  * 最后剩下的占有对象的 shared_ptr 被销毁；
  * 最后剩下的占有对象的 shared_ptr 被通过 operator= 或 reset() 赋值为另一指针。 

用 delete 表达式或在构造期间提供给 shared_ptr 的定制删除器销毁对象。

值得一提的成员变量或者方法：
  1. reset 替换所管理的对象。
  2. swap 交换所管理的对象。
  3. get 返回存储的指针。
  4. use_count  返回 shared_ptr 所指对象的引用计数。
  5. unique 检查所管理对象是否仅由当前 shared_ptr 的实例管理。

值得一提的非成员方法：
  1. make_shared 创建管理新对象的共享指针。

TODO 

## std::mutex C++线程锁

mutex 类是能用于保护共享数据免受从多个线程同时访问的同步原语。

mutex 提供排他性非递归所有权语义：

    调用方线程从它成功调用 lock 或 try_lock 开始，到它调用 unlock 为止占有 mutex 。
    线程占有 mutex 时，所有其他线程若试图要求 mutex 的所有权，则将阻塞（对于 lock 的调用）或收到 false 返回值（对于 try_lock ）.
    调用方线程在调用 lock 或 try_lock 前必须不占有 mutex 。 

若 mutex 在仍为任何线程所占有时即被销毁，或在占有 mutex 时线程终止，则行为未定义。

std::mutex 既不可复制亦不可移动。 

TODO

## RAII(Resource acquisition is initialization)
TODO