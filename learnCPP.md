# C++学习

## cmake 添加c++11支持
set(CMAKE_CXX_STANDARD 11)

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
std::unorderedmap 与std::map的区别
* unordered_map和map类似，都是存储的key-value的值，可以通过key快速索引到value。不同的是unordered_map不会根据key的大小进行排序，
* 存储时是根据key的hash值判断元素是否相同，即unordered_map内部元素是无序的，而map中的元素是按照二叉搜索树存储，进行中序遍历会得到有序遍历。
* 所以使用时map的key需要定义operator<。而unordered_map需要定义hash_value函数并且重载operator==。但是很多系统内置的数据类型都自带这些，
* 那么如果是自定义类型，那么就需要自己重载operator<或者hash_value()了。
* 结论：如果需要内部元素自动排序，使用map，不需要排序使用unordered_map

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
* 基类的虚方法没有设置为纯虚函数，并且没有实现时，编译会报错，要么设为纯虚函数，要么提供实现。
```
undefined reference to `vtable for 虚函数'
undefined reference to `typeinfo for 虚函数'
```

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
* std::reverse()：作用是将输入的两个迭代器之间的元素反序 
## std::array

* 其语义等同于保有一个 C 风格数组 T[N] 作为其唯一非静态数据成员的结构体。
* 该结构体结合了 C 风格数组的性能和可访问性和容器的优点，譬如知晓其大小、支持赋值、随机访问等。
* std::array\<template T, int num\>。

## std::queue

* template\<class T,class Container = std::deque\<T\>\> class queue;
* std::queue 类是容器适配器，它给予程序员队列的功能——尤其是 FIFO （先进先出）数据结构。类模板表现为底层容器的包装器——只提供特定的函数集合。 queue 在底层容器尾端推入元素，从首端弹出元素。 
* 成员函数
  * front()　访问第一个元素
  * back()　访问最后一个元素
  * push()　向队列尾插入元素
  * pop() 删除第一个元素

## std::priority_queue

* template<
    class T,
    class Container = std::vector\<T\>,
    class Compare = std::less\<typename Container::value_type\>\> class priority_queue; 		
* priority_queue 是容器适配器，它提供常数时间的（默认）最大元素查找，对数代价的插入与释出。
* 可用用户提供的 Compare 更改顺序，例如，用 std::greater<T> 将导致最小元素作为 top() 出现。 
```cpp
#include <functional>
#include <queue>
#include <vector>
#include <iostream>
 
template<typename T> void print_queue(T& q) {
    while(!q.empty()) {
        std::cout << q.top() << " ";
        q.pop();
    }
    std::cout << '\n';
}
 
int main() {
    std::priority_queue<int> q;
 
    for(int n : {1,8,5,6,3,4,0,9,7,2})
        q.push(n);
 
    print_queue(q);
 
    std::priority_queue<int, std::vector<int>, std::greater<int> > q2;
 
    for(int n : {1,8,5,6,3,4,0,9,7,2})
        q2.push(n);
 
    print_queue(q2);
 
    // 用 lambda 比较元素。
    auto cmp = [](int left, int right) { return (left ^ 1) < (right ^ 1);};
    std::priority_queue<int, std::vector<int>, decltype(cmp)> q3(cmp);
 
    for(int n : {1,8,5,6,3,4,0,9,7,2})
        q3.push(n);
 
    print_queue(q3);
 
}
```
## constexpr

* 常量表达式(const expression):是指值不会改变，并且在编译过程中就得到计算结果的表达式。
  ``` cpp
  const int i=3;    //是一个常量表达式
  const int j=i+1; //是一个常量表达式
  int k=23;        //k的值可以改变，从而不是一个常量表达式
  const int m=f(); //不是常量表达式，m的值只有在运行时才会获取。
  ```
* C++11允许声明constexpr类型来由编译器检验变量的值是否是一个常量表达式。声明为constexpr的必须是一个常量，并且只能用常量或者常量表达式来初始化。一般来说，若果一旦认定变量是一个常量表达式，那就把它声明为constexpr类型。
  ```cpp
  constexpr int i=3;
  constexpr int j=i+1;
  constexpr int k=f(); //只有f()是一个constexpr函数时k才是一个常量表达式
  ```
* 必须明确一点，在constexpr声明中，如果定义了一个指针，限定符号constexpr仅仅对指针有效，与指针所指对象无关。
  ```cpp
  const int *p=nullptr;  //p是一个指向整型常量的指针（pointer to const）
  constexpr int *p1=nullptr; //p1是一个常量指针(const pointer)
  ```

## 泛型编程与模板

* 非类型模板参数  
  1. 一个非类型参数表示一个值而非一个类型。我们通过一个特定的类型名而非关键字class或typename来指定非类型参数。
  2. 当一个模板被实例化时，非类型参数被一个用户提供的或编译器推断出的值所代替。这些值必须是常量表达式，从而允许编译器在编译时实例化模板。
  ```cpp
  template<unsigned N, unsigned M>
  int compare(const char (&p1)[N], const char (&p2)[M])
  {
    return strcmp(p1,p2);
  }
  ``` 
## boost::bind()

*  boost :: bind是标准函数std :: bind1st和std :: bind2nd的泛化。 它支持任意函数对象，函数，函数指针和成员函数指针，并且能够将任何参数绑定到特定值或将输入参数路由到任意位置。 bind对函数对象没有任何要求; 特别地，它不需要result_type，first_argument_type和second_argument_type标准typedef。
*  bind(f, 1, 2)会产生一个零元函数，它不接受参数并且返回f(1, 2)。类似的，bind(g, 1, 2, 3)( )等同于g(1, 2, 3)。可以选择性地绑定一些参数。 bind（f，_1，5）（x）等价于f（x，5）; 这里_1是一个占位符参数，意思是“用第一个输入参数替换”。
