
# Falcon_2023

2023年开始使用的新AI框架，拥有良好的跨平台能力

## 框架组成

- `3rdParty` -- 编译所用第三方库
- `bin` -- release版软件
	- `base` -- 可执行文件合集
	- `data` -- 重要参数和记录数据
	- `Formation` -- 机器人阵型文件
	- `Help` -- 更多使用教程
	- `LOG` -- log文件
	- `lua_scripts` -- lua脚本
	- `package.bat` -- QT自动打包脚本
	- `run.bat` -- 快捷运行程序脚本
- `cmake` -- cmake编译文件
- `Client` -- 可视化界面、调试工具、视觉处理模块
- `Core` -- 控制、决策算法
- `grSim` -- 物理仿真引擎
- `share` -- 共享代码

## 重构进度

- [x] windows上能编译通过，步骤比较简洁
- [x] 可启动比赛，启动流程比较简单
- [ ] 参数管理优化
	- [ ] 参数共享（Core比较麻烦）
	- [x] Client参数显示可选择文件
- [ ] Client界面改动，采用KDDockWidgets
- [ ] 内置sublime，可在Client上打开脚本
- [ ] 串口通讯和Cray整合（回校后)

## 未解决的顽疾

- [ ] 放大画面的操作放大倍数超过一定数量时，跑Core时会crash，现在限制了放大倍数
- [ ] 多线程之间的信号发射接收有时会有bug，visionmodule和rec_recorder之间

## 编译工具

- 软件
	- [Qt≥5.10](https://www.qt.io/)
	- [CMaket≥3.10](https://cmake.org/download/)
- 依赖库
	- protubuf
	- tolua++
	- lua
	- zlib
	- ode
	- Eigen
	- vartypes

## 编译方法

- Ubuntu : 具体编译方法见B站，搜索"Rocos环境配置",最后再加上下载Vartypes
```bash
$ cd /tmp
$ git clone https://github.com/jpfeltracco/vartypes.git
$ cd vartypes
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```
- Windows : 
	- 注意检查是否有 3rdParty 
	- 使用CMake Gui，输入源代码所在目录和编译目录（一般是源代码所在目录/build）
	- 直接Configure，Qt需要输入路径，如D:\QT\Qt5.12.9\5.12.9\msvc2017_64\lib\cmake\Qt5，其他的都已设置好，无需手动输入
	- 完成后点击Generate，在点击Open Project
	- 在vs中选择Release x64模式，并设置Core项目属性，选择15工具集
	- 开始生成，没有报错则完成
	- 如果需要使用GPU，按照相应教程设置vs工具和项目属性
	- 运行bin/package.bat

## 使用方法

- 启动run.bat

## 常见问题

- 如果软件不正常退出，重新打开后先Kill进程，杀掉未关闭的进程后再开启软件
- 裁判盒指令无法接入，需核查网络配置，虚拟机需将网络设置为桥连模式
- 下车的时候数字键/Ctrl+数字键，待软件反应后再做其他操作，否则容易崩溃
