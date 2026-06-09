# SystemVue ADC C++ 模型

基于 MATLAB `model1` 算法移植的 **SystemVue Data Flow C++ 模型**，兼容 `Envelope`（复包络）信号格式。内核与 SystemVue 框架解耦，可独立编译/测试。

## 架构

```
┌─────────────── SystemVue 框架层 ───────────────┐
│  ADC.h / ADC.cpp                               │
│  · TimedDFModel 派生类                         │
│  · 端口: A_in, A_out, D_I, D_Q                │
│  · 参数声明 + SetHideCondition 条件可见         │
│  · 流式调度: SetRate / SetSampleRate / 跨块状态 │
└──────────────────┬─────────────────────────────┘
                   │ 调用
┌──────────────────▼─────────────────────────────┐
│  ADC_model1.h / ADC_model1.cpp                 │
│  · AdcModel1 纯算法类（无框架依赖）             │
│  · AdcParams 参数结构体 / AdcOutput 输出结构体  │
│  · ProcessClocked()  /  ProcessDownsampled()   │
└──────────────────┬─────────────────────────────┘
                   │ 调用
┌──────────────────▼─────────────────────────────┐
│  spline.h / spline.cpp                         │
│  · CubicSpline: 自然三次样条（Thomas 三对角）   │
│  · ComplexSpline: 对 I/Q 分别建样条            │
└────────────────────────────────────────────────┘
```

## 文件清单

| 文件 | 说明 |
|------|------|
| `ADC.h` | 模型类声明，继承 `TimedDFModel` |
| `ADC.cpp` | 框架接口：`DefineInterface` / `Setup` / `Run` / `PropagateCharacterizationFrequency` |
| `ADC_model1.h` | `AdcModel1` 类声明 + 枚举 + `AdcParams` / `AdcOutput` 结构体 |
| `ADC_model1.cpp` | 核心算法：时钟边沿检测 / 插值 / 量化 / ZOH / 升余弦滤波 |
| `spline.h` | `CubicSpline` / `ComplexSpline` 类声明 |
| `spline.cpp` | 样条实现（Thomas 算法解三对角系统） |
| `LibraryProperties.cpp` | SystemVue 代码生成元数据 |
| `CMakeLists.txt` | CMake 构建配置 |

## 端口

| 端口 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `A_in` | 输入 | `EnvelopeCircularBuffer` | 复包络信号 `I + jQ`，带特征频率 `Fc` |
| `A_out` | 输出 | `EnvelopeCircularBuffer` | 量化后模拟复包络 |
| `D_I` | 输出 | `CircularBuffer<int>` | I 路数字码 |
| `D_Q` | 输出 | `CircularBuffer<int>` | Q 路数字码 |

## 参数

### 通用参数

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `NBits` | 12 | — | ADC 量化位数 |
| `VRef` | 1.0 | V | 参考电压，输入范围 `[-VRef, VRef]` |
| `SR` | 2.4 GHz | Hz | 输入复包络采样率 |
| `CenterFreq` | 0 | Hz | 包络中心频率（0 = 直通 A_in 的 Fc） |
| `OutputDigitalFormat` | Twos-complement | — | 数字输出格式 |
| `ConversionType` | Clocked | — | 转换模式 |
| `InterpMethod` | Spline | — | 插值方式 |

### Clocked 模式参数（`ConversionType = Clocked` 时可见）

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `Clock` | 400 MHz | Hz | 内部余弦时钟频率 |
| `Phase` | 0 | ° | 时钟相位 |

### Downsampled 模式参数（`ConversionType = Downsampled` 时可见）

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `DownsampleFactor` | 1 | — | 降采样因子 |
| `DownsamplePhase` | 0 | — | 降采样相位 |
| `ZohMode` | Integer | — | ZOH 边沿映射方式 |
| `AntiAliasingFilter` | ON | — | 抗混叠滤波使能 |
| `ExcessBW` | 0.5 | — | 升余弦滚降系数（`AntiAliasingFilter = ON` 时可见） |

### 枚举值

**OutputDigitalFormat**
| 值 | 说明 |
|----|------|
| `Twos-complement` | 二进制补码（默认）：`0` = 0V，负码 = 负压 |
| `Offset binary` | 偏移二进制：`2^(N-1)` = 0V |

**ConversionType**
| 值 | 说明 |
|----|------|
| `Clocked` | 时钟采样（默认）：`cos(2π·Clock·t + Phase)` 上升沿触发 |
| `Downsampled` | 降采样：按 `DownsampleFactor` 间隔抽取 |

**InterpMethod**
| 值 | 说明 |
|----|------|
| `Linear` | 线性插值（最快） |
| `PCHIP` | 三次 Hermite 插值（保形） |
| `Spline` | 三次样条插值（最平滑，默认） |

**ZohMode**
| 值 | 说明 |
|----|------|
| `Discretize` | 连续时间对齐：边沿 = `t_samp + 1/SR` |
| `Integer` | 整数索引对齐（默认）：边沿 = `ceil(t_samp × SR + 2)` |

## 流式批处理

### SetRate 机制

```
SetRate(100000)  →  每次 Run() 处理 100k 个采样点
SetSampleRate(SR) → 框架内部时间轴精度 = 1/SR
```

SystemVue 循环调用 `Run()`，每次喂入 `SetRate` 个新采样点。模型通过以下状态保持连续：

```
Run #1:  样本 [0, 99999]      t_start = 0
Run #2:  样本 [100000, 199999] t_start = 100000 × Ts
Run #3:  样本 [200000, 299999] t_start = 200000 × Ts
...
```

### 跨块状态

```cpp
// ADC.h 私有成员
size_t   m_total_offset;  // 全局样本偏移
int      m_last_DI;       // I 路上次保持值
int      m_last_DQ;       // Q 路上次保持值
std::vector<std::complex<double>> m_input_history;  // 输入尾部（padding）
```

- **`m_last_DI / m_last_DQ`**：每块最后输出 → 下块 `AdcParams::init_DI / init_DQ`
- **`m_total_offset`**：累积偏移 → 计算 `t_start = m_total_offset × Ts`
- **`m_input_history`**：保留最近 16 个输入点，供插值 padding

### ResetStreamingState()

构造函数和 `Setup()` 都会调用，确保每次仿真从干净状态开始：

- `m_total_offset = 0`
- `m_input_history.clear()`
- 按格式设初始保持值（补码 = 0，偏移二进制 = 中值）

## 构建

### 环境要求

- Windows x64
- Visual Studio 2022
- SystemVue ModelBuilder SDK（已安装 SystemVue）
- CMake ≥ 3.14

### 编译步骤

```powershell
# 1. 打开 VS 解决方案
cd ADC\build-win64-vs2022
start ADC.sln

# 2. 生成 → Install 项目（右键 → 生成）
# 输出: build-win64-vs2022/output-vs2022/<Config>-SystemVue-Win64/ADC.dll
```

### 编译选项

```cmake
# CMakeLists.txt
add_compile_options("/utf-8" "/O2" "/fp:fast")
```

| 选项 | 说明 |
|------|------|
| `/utf-8` | UTF-8 源文件编码（中文注释） |
| `/O2` | 最大速度优化 |
| `/fp:fast` | 快速浮点模型 |

## 在 SystemVue 中使用

1. **Tools → Library Manager → Add From File** → 选择 `ADC.dll`
2. Part Selector 搜索 `ADC`，拖入原理图
3. 连接 envelope 信号源到 `A_in`
4. 输出 `A_out` / `D_I` / `D_Q` 连接到 sink

## 算法说明

### 时钟采样模式 (Clocked)

```
输入 x(t)  ──→ [时钟边沿检测] ──→ [插值@t_samp] ──→ [量化] ──→ [ZOH] ──→ 输出
                  │                    │
            cos(ωt+φ)=0          Linear/PCHIP/Spline
            上升沿 t0+k·Tclk
```

**时钟边沿检测**：解析 `cos(2π·Clock·t + Phase) = 0` 的上升沿：

```
cos(θ) = 0 且导数 > 0  →  θ = 1.5π + 2kπ
t_samp = (1.5π - φ) / (2π·Clock) + k / Clock
```

**量化**（内联，无 `sqrt`）：

```
I: clamp(xr, -VRef, VRef) → floor((xr+VRef)/LSB) - twos_offset
Q: clamp(xi, -VRef, VRef) → floor((xi+VRef)/LSB) - twos_offset
```

**ZOH 保持**：每采样点的输出 = 最近一次时钟边沿的量化值。

**分块处理**：100k 采样点/块，5 点 padding 重叠，避免边界效应。

### 降采样模式 (Downsampled)

```
输入 x(t)  ──→ [抗混叠滤波(可选)] ──→ [抽取@Factor,Phase] ──→ [量化] ──→ [ZOH] ──→ 输出
               │
         升余弦 FIR, span=20
```

- 可选升余弦抗混叠滤波（`ExcessBW` 控制滚降）
- 按 `DownsampleFactor` 间隔抽取，`DownsamplePhase` 偏移起始点
- ZOH 保持 `Factor-1` 个点直到下次采样

## 性能优化

针对百万级采样点的场景做了以下优化：

| # | 优化 | 方法 | 提升 |
|---|------|------|------|
| 1 | 预分配向量复用 | chunk 循环内 `resize()` 代替重新分配 | ~15-25% |
| 2 | 零拷贝引用 | Clocked 模式直接传 `const &`，不拷贝 | ~5-10% |
| 3 | 量化内联 + 去 `sqrt` | I/Q 独立限幅，消除 `std::abs` 的 `sqrt` | ~20-30% |
| 4 | 常量外提 | `inv_LSB`, `twos_offset`, `±VRef` 提至循环外 | ~5-10% |
| 5 | 滑动窗口 | 替代 `std::upper_bound`，均摊 O(1) | ~5-10% |

- **综合提升**：约 2-3×
- **最大受益路径**：Clock 模式 + Spline 插值

详见源码 `ADC_model1.cpp` 内注释。

## 与 Atod 对比

| 维度 | 本模型 | SystemVue Atod |
|------|--------|---------------|
| 时钟模式 | `cos(2πft+φ)` 解析过零点 | 内置时钟引擎 |
| 降采样 | 支持，可变因子/相位 | 支持 |
| 抗混叠 | 升余弦 FIR (span=20) | 内置 |
| 插值 | Linear / PCHIP / Spline | 固定方式 |
| 数字格式 | 补码 / 偏移二进制 | 两者 |
| 性能 | ~5s/10M 点 (Spline) | ~2.7s/10M 点 |
| 可定制性 | 源码可改 | 封闭 |

## 许可

MIT License — 详见仓库根目录。
