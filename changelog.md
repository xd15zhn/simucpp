# CHANGELOG
## V2.1.1
- [packmodules.cpp] `StateTransFcn`删除$D$矩阵.
- [simulator.cpp] ADDED: `FLAG_REDUNDANT`.
- [matmodules.cpp/hpp] ADDED: `MOutput`.

## V2.1.2
- [matmodules.cpp/hpp] BUGFIXED: `MOutput`.

## V2.1.3
- [unitmodules.cpp/hpp，matmodules.cpp/hpp] CHANGED: 函数指针改为`std::function`.

## V2.1.4
- [matmodules.cpp] BUGFIXED: lambda表达式改为按值捕获.
- [debugs.cpp] CHANGED: 添加打印3个模块的输入端口增益.
- [simulator.cpp/hpp] ADDED: `Print_Connection`输出检测出的代数环.

## V2.1.5
- [baseclass.hpp/unitmodules.hpp] CHANGED: 部分注释适配doxygen。
- [baseclass.hpp/unitmodules.cpp/hpp] CHANGED: 部分int类型改为uint。

## V2.1.6
- [unitmodules.cpp/hpp] DELETED: `Print_DebugInfo`.改为指针类型强制转换。
- [simulator.cpp/hpp] CHANGED: `Print_Connection`改为局部函数.
- [debugs.cpp] DELETED: 删除文件，将函数移入`simulator.cpp`.

## V2.1.7
- [simulator.cpp/hpp] DELETED: `Simulate_FinalStep`.
- [unitmodules.cpp/hpp] DELETED: 因为没找到合适的算法，所以暂时取消删除多余模块的功能.
- [baseclass.hpp] DELETED: `Set_Enable`.
- CHANGED: 所有应该使用uint的int类型改为uint。
- [simulator.cpp/hpp] CHANGED: `_outref`改为数组.

## V2.1.8
- [simulator.cpp/hpp] ADDED: `Simulate_FinalStep`.
- [matmodules.hpp] CHANGED.
- [baseclass.hpp] CHANGED.

## V2.1.9
- [unitmodules.hpp] DELETED: 默认自定义函数.
- [unitmodules.hpp] BUGFIXED: `_enabled`.

## V2.1.10
- [matmodules.hpp] BUGFIXED: `MStateSpace::_next`.
- [matmodules.cpp/hpp] DELETED: `MProduct`,`MTranspose`.
- [packmodules.hpp] DELETED: `ProductScalarMatrix`.
- [unitmodules.hpp/simulator.cpp] CHANGED: 修改部分注释.

## V2.1.11
- [baseclass.hpp] ADDED: `Get_UnitModule()`.
- [baseclass.hpp] ADDED: `Get_ID()`.
- [packmodules.cpp] BUGFIXED: `delete`,`c_str()`.

## V2.1.12
- CHANGED: `uint` to `uint32_t`.
- [simulator.cpp/hpp] CHANGED: `Simulator()`, `Simulate(double endtime)`.
- [simulator.cpp/hpp] CHANGED: `Print_Modules()` to `Print_Modules_Information()`.
- [simulator.cpp/hpp] ADDED: `Print_Modules_SequenceTable()`.
- [simulator.cpp/hpp] CHANGED: `Initialize(uint32_t print)`.
