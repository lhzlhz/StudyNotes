# C++学习

## std::shared_ptr<>
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
  2. allocate_shared 创建共享指针，管理用分配器分配的新对象。 

注意：
  * 只能通过复制构造或复制赋值其值给另一 shared_ptr ，将对象所有权与另一 shared_ptr 共享。用另一 shared_ptr 所占有的底层指针创建新的 shared_ptr 导致未定义行为。 
 

## std::mutex C++线程锁

mutex 类是能用于保护共享数据免受从多个线程同时访问的同步原语。

mutex 提供排他性非递归所有权语义：

    调用方线程从它成功调用 lock 或 try_lock 开始，到它调用 unlock 为止占有 mutex 。
    线程占有 mutex 时，所有其他线程若试图要求 mutex 的所有权，则将阻塞（对于 lock 的调用）或收到 false 返回值（对于 try_lock ）.
    调用方线程在调用 lock 或 try_lock 前必须不占有 mutex 。 

若 mutex 在仍为任何线程所占有时即被销毁，或在占有 mutex 时线程终止，则行为未定义。

std::mutex 既不可复制亦不可移动。 

注意：
  * 通常不直接使用std::mutex，而是使用std::unique_lock或者std::lock_guard以更加异常安全的方式管理锁定。

## std::lock_guard:

  类 lock_guard 是互斥封装器，为在作用域块期间占有互斥提供便利 RAII 风格机制。

  创建 lock_guard 对象时，它试图接收给定互斥的所有权。控制离开创建 lock_guard 对象的作用域时，销毁 lock_guard 并释放互斥。

  lock_guard 类不可复制。 

```cpp
#include <thread>
#include <mutex>
#include <iostream>
 
int g_i = 0;
std::mutex g_i_mutex;  // 保护 g_i
 
void safe_increment()
{
    std::lock_guard<std::mutex> lock(g_i_mutex);
    ++g_i;
 
    std::cout << std::this_thread::get_id() << ": " << g_i << '\n';
 
    // g_i_mutex 在锁离开作用域时自动释放
}
 
int main()
{
    std::cout << "main: " << g_i << '\n';
 
    std::thread t1(safe_increment);
    std::thread t2(safe_increment);
 
    t1.join();
    t2.join();
 
    std::cout << "main: " << g_i << '\n';
}
```


## RAII(Resource acquisition is initialization)

一种编程风格(或思想)。核心思想就是，用类来管理资源，当类初始化时，该资源被获得，当类被销毁时，该资源被释放，这样就保证了只要没有对象泄露，则资源就不会泄露(If there are no object leaks, there are no resource leaks)。

```cpp
#include <mutex>
#include <iostream>
#include <string> 
#include <fstream>
#include <stdexcept>

void write_to_file (const std::string & message) {
    // mutex to protect file access (shared across threads)
    static std::mutex mutex;

    // lock mutex before accessing file
    std::lock_guard<std::mutex> lock(mutex);

    // try to open file
    std::ofstream file("example.txt");
    if (!file.is_open())
        throw std::runtime_error("unable to open file");
    
    // write message to file
    file << message << std::endl;
    
    // file will be closed 1st when leaving scope (regardless of exception)
    // mutex will be unlocked 2nd (from lock destructor) when leaving
    // scope (regardless of exception)
}

```

## 继承、基类与派生类

* protect 关键字保证除了该类的继承类可以访问这些成员，其他用户无法访问。
* 派生类的类派生列表(class derivation list)：由一个冒号和一个以逗号分隔的基类列表组成。每个基类可以有三种访问说明符:public、protected、private。这些说明符决定了该基类的public成员与方法在派生类中的可见程度。
* 基类的引用或者指针可以指向派生类的实例，而由这个引用或者指针调用基类方法时会调用指向的派生类的实例的方法。

## std::vector

值得注意的成员方法
* std::vector::front() 返回容器中第一个元素的引用。
* std::vector::back() 返回容器中最后一个元素的引用。
* std::vector::rend() reverse end 返回容器中反向迭代器的最后一个迭代器。

值得注意的非成员方法

* std::upper_bound()
  * 函数原型 template< class ForwardIt, class T >
ForwardIt upper_bound( ForwardIt first, ForwardIt last, const T& value );
  * 该方法在迭代器first与last中找到第一个比value大的成员，并且返回其迭代器，若没有则返回last的迭代器。