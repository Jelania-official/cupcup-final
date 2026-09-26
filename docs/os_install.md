# 环境配置教程
![Relative date](https://img.shields.io/date/1569071485)

*   本教程在匆忙中完成，必然有诸多疏漏。如有意见或建议，可以通过[issue](https://gitee.com/robocup/SEURoboCup2021/issues)或者QQ群向我们提供反馈。
*   本教程有许多操作可以通过网上教程知悉，此处暂且不表，一律以“略”代替。


## Ubuntu 20.04安装

**！！数据无价，切记备份！！**

**！！数据无价，切记备份！！**

**！！数据无价，切记备份！！**

[完整安装参考文章链接](https://blog.csdn.net/qq_45488453/article/details/107783752?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522162894330516780255265662%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=162894330516780255265662&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_v2~times_rank-2-107783752.first_rank_v2_pc_rank_v29&utm_term=ubuntu20.04%E5%8F%8C%E7%B3%BB%E7%BB%9F%E5%AE%89%E8%A3%85&spm=1018.2226.3001.4187)

**推荐在CSDN上多看几篇ubuntu系统安装的教程**


1. 您需要准备一个**不小于4GB的无用U盘（数据将会被清除！）**，一台在磁盘尾部有**不小于40GB空余空间的电脑**（内存应不少于8GB，有独显更好）。

2. [下载镜像](https://mirrors.tuna.tsinghua.edu.cn/ubuntu-releases/20.04/ubuntu-20.04.2.0-desktop-amd64.iso)
> 请注意，镜像虽然实质是一个压缩包，但是不需要解压缩。

3. 为了给 *Ubuntu* 腾出40GB空间，您通常需要压缩磁盘卷。略。
Note:右键开始logo→磁盘管理

4. 烧写工具推荐使用[*Rufus*](https://rufus.ie/)。烧写时通常需要注意引导方式为UEFI。


5. 将U盘烧写为安装盘后，您需要重启电脑，将电脑设置为U盘启动。略。

6. 启动后，您需要在您之前腾出的不小于40GB的磁盘空间上安装 *Ubuntu 20.04。
***
以下操作十分危险，请慎重！曾有同学在此步不慎清空了有用的数据！

  * **请在选择 *安装类型* 时务必选择 *其它选项* ！！**多硬盘用户请注意启动项安装位置。

  * 进入其它选项后，选中您先前预留的磁盘空间，点击右下角的`+`号。如果您在分配该磁盘空间时不慎将其格式化为了NTFS等文件格式，此处请先点击`-`号。
  > 注意：`-`号意味着该磁盘空间上所有数据都将被清空！如果您不慎选错磁盘空间，请立即停止操作，不要点击`现在安装`，而是点击`退出`或`后退`，此时由于并未应用新的分区表，您的数据得以保留而未被格式化。

**最好保证系统安装一遍过**
**分区是系统安装的关键一步,根据我安装的经验，推荐按照我以下的分区（以50G空间为例）**   

这里我们要分四个区域，分别是
- /: 根目录,整个系统的大区域，条件允许可以大一点，可以20G，毕竟ubuntu装软件默认是装在这里的，大一点可能会省去后面隐藏的麻烦。推荐25G
- swap: 交换空间，一般和内存一样大。推荐8G
- /boot：启动引导分区，这个就是用于启动 ubuntu 的目录，里面会有系统的引导，建议将其划分为 1G 文件格式为 ext4。
- /home:这是 ubuntu 的“其他盘”， 这个也可以说是我们的个人目录，剩下的大小都分给它。

 /   根目录  大小25G 主分区 空间起始位置 Ext4 (图片参考)
![输入图片说明](https://img-blog.csdnimg.cn/2020103111435189.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Ftb3J4MTIzNDU=,size_16,color_FFFFFF,t_70#pic_center "在这里输入图片标题")

swap 交换空间 大小8G 逻辑分区 空间起始位置
![输入图片说明](https://img-blog.csdnimg.cn/20201031114458842.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Ftb3J4MTIzNDU=,size_16,color_FFFFFF,t_70#pic_center "在这里输入图片标题")

/boot 启动目录 大小1G 逻辑分区 空间起始位置 Ext4
![输入图片说明](https://img-blog.csdnimg.cn/2020103111483099.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Ftb3J4MTIzNDU=,size_16,color_FFFFFF,t_70#pic_center "在这里输入图片标题")

/home 家目录 大小剩下都给他 逻辑分区 空间起始位置 Ext4
![输入图片说明](https://img-blog.csdnimg.cn/20201031114919745.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Ftb3J4MTIzNDU=,size_16,color_FFFFFF,t_70#pic_center "在这里输入图片标题")

 **"安装启动引导器的设备"选择 **/boot** 对应的分区，检查无误后选择“现在安装”，再选择“继续”。** 
![输入图片说明](https://img-blog.csdnimg.cn/20200711104735596.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80NDg1NzY4OA==,size_16,color_FFFFFF,t_70 "在这里输入图片标题")

 


只需一杯咖啡的时间，您的 *Ubuntu* 就已准备就绪😊。

安装完成后，您的电脑需要重新启动，并且通常会默认重启到全新安装的系统中。如果要切换启动项或者更改启动项顺序和默认启动项，请进入BIOS设置。略。

### 换源

进入全新安装的 *Ubuntu 20.04* 后，您通常需要更换源为国内源。该项操作推荐进入*软件和更新*中，在*下载自*下拉栏中选择*清华、阿里、华为*等镜像站，或者点击*选择最佳服务器*并等待一会。。



> 如果您不知道终端是什么，您可能需要恶补Linux系统使用方法。为了表示友好，这里提示您终端的快捷键是`Ctrl+Alt+T`，或者在文件夹中右键。


## 其它

这里推荐使用 *Visual Studio Code* 浏览和编辑代码。
[Ubuntu下安装VS Code](https://blog.csdn.net/shuiyixin/article/details/98970992?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522162894751116780274193538%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=162894751116780274193538&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_v2~hot_rank-2-98970992.first_rank_v2_pc_rank_v29&utm_term=ubuntu+vscode%E5%AE%89%E8%A3%85&spm=1018.2226.3001.4187)

## 谢谢~
