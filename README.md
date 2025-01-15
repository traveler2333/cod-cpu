# 五级流水线 CPU 项目

## 项目概述

本项目实现了一个五级流水线 CPU，包含以下功能模块：

- 数据冲突
- 分支预测
- 异常处理
- 页表
- TLB（Translation Lookaside Buffer）
- Icache（指令缓存）
- Dcache（数据缓存）

## 模块说明

### 数据冲突
处理流水线中可能出现的数据依赖问题，确保指令正确执行。

### 分支预测
提高流水线效率，通过预测分支指令的执行路径减少流水线停顿。

### 异常处理
处理 CPU 执行过程中可能出现的异常情况，确保系统稳定运行。

### 页表
实现虚拟地址到物理地址的映射，支持内存管理。

### TLB
加速虚拟地址到物理地址的转换，提高内存访问效率。

### Icache
缓存指令，提高指令获取速度，减少内存访问延迟。

### Dcache
缓存数据，提高数据访问速度，减少内存访问延迟。

## 编译与运行

工程包含示例代码和所有引脚约束，可以直接编译。

代码中包含中文注释，编码为 UTF-8，在 Windows 版 Vivado 下可能出现乱码问题。  
请用别的代码编辑器打开文件，并将编码改为 GBK。

## 贡献者

感谢所有为本项目做出贡献的成员。

## 许可证

本项目遵循 MIT 许可证。