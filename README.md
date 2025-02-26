# CoppeliaSimEdu-Based-ImgRecognition-PIDcontrol
A code based on CoppeliaSim Edu software that uses image recognition to automatically find the way for a car and control it to move along the calculated path

一个基于CoppeliaSim Edu软件，通过图像识别实现小车自动寻路并控制小车沿着计算路径前进的代码

## 运行环境
python 3.9  
Windows 10  
CoppeliaSim Edu 4.6.0

## 运行代码
1. 首先要配置CoppeliaSim Edu软件与python代码的远程连接，打开CoppeliaSim Edu软件的根目录下的
`\CoppeliaSimEdu\remoteApiConnections.txt`文件，检查portIndex1_port后等于的数值，要与
`main-PIDsys.py`和`main-PurePursuit.py`文件中`clientID=sim.simxStart()`中的第二个参数一样。
2. 接着打开CoppeliaSim Edu软件的根目录，将`CoppeliaSimEdu\programming\legacyRemoteApi\remoteApiBindings\python\python`
下的`sim.py`, `simConst.py`, `simpleSynchronousTest.py`复制到本项目根目录下。
3. 然后将`CoppeliaSimEdu\programming\legacyRemoteApi\remoteApiBindings\lib\lib\Windows`
下的`remoteApi.dll`也复制到本项目根目录下。
4. 接下来请用CoppeliaSim Edu软件打开对应的模拟环境（本项目中为`scene.ttt`）。
5. 最后运行`main-PIDsys.py`和`main-PurePursuit.py`两个文件之一即可。(在代码开头，可以修改h_method参数，
选择不同的A*算法启发式或选择Dijkstra方法)

## 代码介绍
本项目主要实现的代码有三个，分别是`main-PIDsys.py`，`main-PurePursuit.py`以及`tools.py`。
其中`tools.py`中，实现了一个路径搜索类完成A*搜索和DIjkstra搜索，以及一个PID系统类。   
而`main-PIDsys.py`和`main-PurePursuit.py`两个文件则调用`tools.py`完成路径搜索并实现轨迹跟踪，
二者的区别从命名上也能略知一二，主要是轨迹跟踪的算法不同。
