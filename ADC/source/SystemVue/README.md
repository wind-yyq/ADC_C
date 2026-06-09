# SystemVue ADC C++ 模型

基于 MATLAB `model1` 算法移植的 SystemVue C++ Data Flow 模型，兼容 `Envelope`（复包络）信号格式。

## 项目结构

```
SystemVue/
├── ADC.h                # 模型类声明（继承 TimedDFModel）
├── ADC.cpp              # SystemVue 框架接口：DefineInterface / Setup / Run / PropagateCharacterizationFrequency
├── ADC_model1.h         # 核心算法类 AdcModel1 声明 + 枚举 + AdcParams / AdcOutput 结构体
├── ADC_model1.cpp       # 核心算法实现：时钟采样 / 降采样 / 插值 / 量化 / 升余弦滤波
├── spline.h             # 三次样条插值类 (CubicSpline / ComplexSpline)
├── spline.cpp           # 样条实现（Thomas 算法解三对角系统）
├── LibraryProperties.cpp
├── CMakeLists.txt       # CMake 构建配置（含 /utf-8 编译选项）
└── README.md
```

## 端口

| 端口 | 方向 | 类型 | 说明 |
|------|------|------|------|
| A_in | 输入 | `EnvelopeCircularBuffer` | 复包络信号 (I+jQ) |
| A_out | 输出 | `EnvelopeCircularBuffer` | 采样后模拟复包络 |
| D_I | 输出 | `CircularBuffer<int>` | I 路数字码 |
| D_Q | 输出 | `CircularBuffer<int>` | Q 路数字码 |

## 参数

| 参数 | 默认值 | 类型 | 说明 | 可见条件 |
|------|--------|------|------|---------|
| NBits | 12 | Float | ADC 位数 | 始终 |
| VRef | 1.0 | Float (V) | 参考电压 | 始终 |
| SR | 24e6 | Float (Hz) | 输入采样率 | 始终 |
| CenterFreq | 0 | Float (Hz) | 包络中心频率 | 始终 |
| OutputDigitalFormat | Twos-complement | Enum | 数字输出格式 | 始终 |
| ConversionType | Clocked | Enum | 转换模式 | 始终 |
| Clock | 10e6 | Float (Hz) | 采样时钟频率 | Clocked 模式 |
| Phase | 0.0 | Float (deg) | 时钟相位 | Clocked 模式 |
| InterpMethod | Spline | Enum | 插值方式 (Linear/PCHIP/Spline) | 始终 |
| DownsampleFactor | 1 | Float | 降采样因子 | Downsampled 模式 |
| DownsamplePhase | 0 | Float | 降采样相位 | Downsampled 模式 |
| ZohMode | Integer | Enum | ZOH 边沿映射 | Downsampled 模式 |
| AntiAliasingFilter | ON | Enum | 抗混叠滤波使能 | Downsampled 模式 |
| ExcessBW | 0.5 | Float | 升余弦滚降系数 | AA=ON 时 |

## 构建

1. 用 Visual Studio 打开 `build-win64-vs2022/ADC.sln`
2. CMake 自动 configure，生成 `SystemVue-ADC` 项目
3. 右键 Install 项目 → 生成
4. 生成的 DLL 位于 `build-win64-vs2022/output-vs2022/Debug-SystemVue-Win64/ADC.dll`

## 在 SystemVue 中使用

1. Tools → Library Manager → Add From File → 选择 `ADC.dll`
2. Part Selector 里找到 `ADC` 模型
3. 连接 envelope 源到 A_in，输出 A_out / D_I / D_Q 到 sink

## 算法说明

### 时钟采样 (Clocked)
- 检测 cos(2π·Clock·t + Phase) 的过零点作为采样时刻
- 插值（Linear / PCHIP / Spline）获取采样点信号值
- 量化（TwosComplement / OffsetBinary）
- ZOH 保持到输出网格

### 降采样 (Downsampled)
- 可选的升余弦抗混叠滤波
- 按 DownsampleFactor 和 DownsamplePhase 抽取样本
- 量化后输出

## 与 Atod 对比

稳态输出与 SystemVue 内置 Atod 模型完全一致。初始几个采样点存在微小偏差（~2 LSB），根因定位为 `use_fine_clk`（Clock > SR/4）路径的时钟边沿检测逻辑。

## 框架特性

- `TimedDFModel`：支持包络信号时间管理
- `Setup()`：端口 SetRate 配置
- `PropagateCharacterizationFrequency()`：输入包络 Fc 直通到输出，或由 CenterFreq 参数覆盖
- `SetHideCondition`：参数条件显示（Clocked/Downsampled 模式下自动切换可见参数）
- `/utf-8`：MSVC UTF-8 源文件编译支持
