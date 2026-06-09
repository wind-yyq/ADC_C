// ADC_model1.h — ADC 核心算法接口（与 SystemVue 框架无关）
#pragma once
#include <vector>
#include <complex>

// ===================== 枚举类型 =====================

/// 转换模式
enum ConvType
{
	Clocked = 0,	// 时钟采样（默认）
	Downsampled = 1 // 降采样
};

/// 插值方式
enum InterpType
{
	Linear = 0, // 线性插值
	PCHIP = 1,	// 三次 Hermite 插值
	Spline = 2	// 三次样条插值（默认）
};

/// 数字输出格式
enum OutFmtType
{
	TwosComplement = 0, // 二进制补码（默认）
	OffsetBinary = 1	// 偏移二进制
};

/// ZOH 边沿映射模式
enum ZohMappingType
{
	Discretize = 0, // 连续时间对齐
	Integer = 1		// 整数索引对齐（默认）
};

/// 抗混叠滤波使能
enum AntiAliasingMode
{
	AA_OFF = 0, // 关闭
	AA_ON = 1	// 开启（默认）
};

// ===================== 参数 / 输出结构体 =====================

struct AdcParams
{
	int n_bits = 12;								   // ADC 位数
	double v_ref = 1.0;								   // 参考电压 (V)
	double sr = 2.4e9;								   // 输入采样率 (Hz)
	double center_freq = 0.0;						   // 包络中心频率 (Hz)
	double clock = 4.0e9;							   // 采样时钟 (Hz)
	double phase = 0.0;								   // 时钟相位 (°)
	ConvType conversion_type = Clocked;				   // 转换模式
	InterpType interp_method = Spline;				   // 插值方式
	OutFmtType output_digital_format = TwosComplement; // 输出格式
	ZohMappingType zoh_mode = Integer;				   // ZOH 映射
	int downsample_factor = 1;						   // 降采样因子
	int downsample_phase = 0;						   // 降采样相位
	AntiAliasingMode anti_aliasing_filter = AA_ON;	   // 抗混叠使能
	double excess_bw = 0.5;							   // 升余弦滚降系数
};

struct AdcOutput
{
	std::vector<std::complex<double>> a_out; // 模拟输出复包络
	std::vector<int> d_i;					 // I 路数字码
	std::vector<int> d_q;					 // Q 路数字码
	std::vector<double> t_out;				 // 输出时间点
};

// ===================== AdcModel1 类 =====================

class AdcModel1
{
public:
	/// 主入口（自动分发 Clocked / Downsampled）
	/// x_in: 输入复包络 (I + jQ)
	/// Ts:   输入采样周期 (1/SR)
	AdcOutput Process(const std::vector<std::complex<double>> &x_in,
					  double Ts,
					  const AdcParams &params);

private:
	// ---- 模式分发 ----
	AdcOutput ProcessClocked(const std::vector<double> &t,
							 const std::vector<std::complex<double>> &x,
							 const AdcParams &params);

	AdcOutput ProcessDownsampled(const std::vector<double> &t,
								 const std::vector<std::complex<double>> &x,
								 const AdcParams &params);

	// ---- 内部工具函数 ----
	static double deg2rad(double deg);
	static int clamp_int(int x, int lo, int hi);
	static double clamp_double(double x, double lo, double hi);
	static int quantize_one(double x, double VRef, double LSB,
							int levels, int NBits, bool twos_complement);
	static double catmull_rom_1d(const std::vector<double> &x,
								 const std::vector<double> &y, double xx);
	static void interp_catmull_rom(const std::vector<double> &t,
								   const std::vector<std::complex<double>> &x,
								   const std::vector<double> &t_samp,
								   std::vector<std::complex<double>> &out);
	static void apply_raised_cosine(std::vector<std::complex<double>> &y,
									double beta, double sps, bool is_complex);
};
