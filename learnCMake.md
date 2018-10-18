# CMake 学习

## 前言

我觉的基础知识还是要打好的，所以学这个了。教材是[An introduction to modern CMake](http://cliutils.gitlab.io/modern-cmake/chapters/intro/running.html).

以下每个副标题都是一个章节的知识点。

## Running CMake

1. cmake .. & make = cmake --build
2. 在bash中选择clang作为编译器
  ```shell
  $ CC=clang CXX=clang++ cmake
  ```
3. 一些标准设置
  * *-DCNAKE_BUILD_TYPE=* 可以从Release, RelWithDebInfo, Debug 或者其他的中选。
  * *-DCMAKE_INSTALL_PREFIX=* 要被安装在哪个文件夹。UNIX下通常是/usr/local。用户文件夹通常是~/.local,或者你可以自己选一个。
  * *-D BUILD_SHARED_LIBS=* 可以选择ON或者OFF来选择是否生成共享库文件。
  * --trace 选项会打印出cmake运行中的中的每一行。--trace-source="filename"会输出到文件中。

## DO\'s and Don\'ts
    
1. Ubuntu 16.04的cmake版本是3.5.1。
2. 反对的CMake书写模式
   * **不要使用全局函数**：包括*link_directories*、*include_libraries*等等。
   * **不要增加不需要的公共的需求**：你应该避免强迫使用者使用不必要的模式(-Wall)。让它们成为private。
   * **Dont't GLOB files**:要在增加文件之后rerun CMake.
   * **直接链接文件**。
   * **链接时不要忽略PUBLIC/PRIVATE**：会导致未来的链接缺乏关键词。
