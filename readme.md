# 学习笔记（们）

考虑到Ubuntu环境下学习的深刻危险性（随时要因为各种不可抗力重装系统），决定借git存一下笔记了。

## 更新笔记

* 10.3更新
    * 妈类昨晚刚把学习笔记上传，然后系统又崩了，花了一天多又重新配了环境。记下Hasee战神系列笔记本配置环境的坑爹之处。
        * 首先，安装时 e编辑grub 在quite splash --- 后，去掉 '---'， 添加nomodeset以屏蔽Nvidia独显。
        * 其次，安装完后，你还是要e编辑grub的 ubuntu菜单在quite splash 后添加nomodeset，不然会黑屏
        * 然后，在dock里搜索software&update，安装它提供的驱动。
        * 重启后就不要'nomodeset'选项了，进入，sudo vim /etc/default/grub，删除nomodeset, update-grub
    * 然后开机自启动程序的话，dock搜索startup application就可以了。
    * firefox的flashplayer插件的存放地址为/usr/lib/mozilla/plugins/

* 10.8更新
    * anaconda包和系统原生的python包含路径好像是会冲突的，见下
    >  I have anaconda library installed in my home directory. So cmake is detecting intel MKL and other libraries from that folder. i have updated my LD_LIBRARY_PATH variable to include that folder and the issues are resolved.
    >Just add the following line to /.bashrc in case you have anaconda installed in folder

    >export LD_LIBRARY_PATH=/path_to_lib_folder/lib:$LD_LIBRARY_PATH