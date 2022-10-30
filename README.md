# SIMUCPP

C++重写simulink.  
代码仓库 <https://github.com/xd15zhn/simucpp>  
<https://gitee.com/xd15zhn/simucpp>  
教程博客 <https://blog.csdn.net/qq_34288751/category_11453352.html>  
例程 <https://gitee.com/xd15zhn/simucpp_demo>  
博客原文 <https://gitee.com/xd15zhn/simucpp_blog>  

有两个分支，master是频繁修改更新的分支，不一定可用；release是经过测试可以使用的稳定版。  
release:稳定发布版本：V2.1.0  

# 依赖库
- <https://gitee.com/xd15zhn/tracelog> (选装)
- <https://gitee.com/xd15zhn/matplotlibcpp> (选装)
- <https://gitee.com/xd15zhn/zhnmat> (选装)

# 博客目录
[simucpp：C++搭建微分方程求解器框架(重写simulink)](https://blog.csdn.net/qq_34288751/article/details/117740605)  
[simucpp系列教程(1)安装教程](https://blog.csdn.net/qq_34288751/article/details/121111051)  
[simucpp系列教程(2)例程解析(第一部分)](https://blog.csdn.net/qq_34288751/article/details/121112003)  
[simucpp系列教程(3)例程解析(第二部分)](https://blog.csdn.net/qq_34288751/article/details/122155363)  
[simucpp系列教程(4)使用教程与程序说明](https://blog.csdn.net/qq_34288751/article/details/122285634)  
[simucpp系列教程(5)各模块的简要介绍](https://blog.csdn.net/qq_34288751/article/details/122724035)  
[simucpp系列教程(6)函数文档](https://blog.csdn.net/qq_34288751/article/details/123325005)  
[番外篇(1)模块次序表、代数环及其检测算法](https://blog.csdn.net/qq_34288751/article/details/122648967)  
[番外篇(2)连续离散混合仿真中的若干问题](https://blog.csdn.net/qq_34288751/article/details/122708048)  
[番外篇(3)矩阵模块与复用模块的设计细节](https://blog.csdn.net/qq_34288751/article/details/126324078)  

# 目前用该库解决的实用问题
- [倒立摆控制与仿真动画展示](https://blog.csdn.net/qq_34288751/article/details/122640930)
- [卫星轨道3D仿真程序](https://gitee.com/xd15zhn/satellitesim)
- [降维状态观测器的直观理解与仿真](https://blog.csdn.net/qq_34288751/article/details/123061451)
- [导弹追踪问题](https://blog.csdn.net/qq_34288751/article/details/122725421)
- [水流冲重物问题](https://blog.csdn.net/qq_34288751/article/details/123159543)

# 版本说明
当前为V2版本，V1版本已废弃，代码见<https://gitee.com/xd15zhn/simucpp-V1>。次版本号的发布版在release中。

# 一个简单的例子
```cpp
// main.cpp
/*一阶系统阶跃响应*/
#include <iostream>
#include "simucpp.hpp"
using namespace simucpp;
using namespace std;

double Input_Function(double t)
{
    return 1;
}

int main()
{
    // 初始化仿真器
    Simulator sim1(10);

    // 初始化模块
    UIntegrator *integrator1 = new UIntegrator(&sim1);
    USum *sum1 = new USum(&sim1);
    UInput *input1 = new UInput(&sim1);
    UOutput *output1 = new UOutput(&sim1);

    // 模块之间的连接
    sim1.connectU(input1, sum1);
    sum1->Set_InputGain(1);
    sim1.connectU(sum1, integrator1);
    sim1.connectU(integrator1, sum1);
    sum1->Set_InputGain(-0.5);
    sim1.connectU(integrator1, output1);

    // 参数设置
    integrator1->Set_InitialValue(0);
    input1->Set_Function(Input_Function);

    // 运行仿真
    sim1.Initialize();
    sim1.Simulate();
    cout.precision(12);
    cout << output1->Get_OutValue() << endl;
    sim1.Plot();
    return 0;
}
```
```
# CMakeLists.txt
cmake_minimum_required(VERSION 3.12)
project(untitled)
set(CMAKE_BUILD_TYPE release)
add_executable(${CMAKE_PROJECT_NAME} main.cpp)
list(APPEND CMAKE_PREFIX_PATH "E:/cpplibraries/")
find_package(simucpp REQUIRED)
message(STATUS "simucpp_VERSION: ${simucpp_VERSION}")
message(STATUS "simucpp_DIR: ${simucpp_DIR}")
message(STATUS "simucpp_LIBS: ${simucpp_LIBS}")
message(STATUS "simucpp_INCLUDE_DIRS: ${simucpp_INCLUDE_DIRS}")
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC ${simucpp_LIBS})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${simucpp_INCLUDE_DIRS})
```
