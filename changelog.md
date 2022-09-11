# V2.0.1
- 添加`tracelog`日志库。
- 删除`SIMUCPP_ASSERT_ERROR`和`SIMUCPP_ASSERT_WARNING`。

# V2.0.2
- [CMakeLists.txt] BUGFIXED.
- 添加一些注释，优化代码格式等。

# V2.0.3
- [simulator.cpp/hpp] RENAME: `duration` -> `endtime`.
- [packmodules.cpp] BUGFIXED.
- 添加一些日志打印。

# V2.0.4
- [CMakeLists.txt] BUGFIXED: `target_include_directories`.
- [CMakeLists(linux).txt] DELETED.
- [definitions.hpp] BUGFIXED: `c_str`.

# V2.0.5
- [simulator.cpp/hpp] DELETED: `Set_WarningLevel`.
- [simulator.cpp/hpp] DELETED: `VERSION`.
- [UNoise] CHANGED: 重写正态分布随机变量生成函数。

# V2.0.6
- [unitmodules.cpp/simulator.cpp] BUGFIXED: `c_str`.
- [simulator.hpp] RENAMED: `Set_DivergenceCheckMode` -> `Set_PassNaN`.
- [simulator.hpp] DELETED: `Set_PassNaN`.
- [unitmodules.cpp] CHANGED: 将所有直通模块的初始输出值设为`NaN`。

# V2.0.7
- [matmodules.cpp/simulator.cpp] ADDED: `MProduct`.
- [matmodules.cpp] BUGFIXED: if (!(size<_size)).
- [matmodules.cpp] DELETED: 仿真器的`connectU`和`connectM`函数删除默认连接端口.
