// ADC.cpp — SystemVue 框架接口
#include "ADC.h"

#ifndef SV_CODE_GEN
DEFINE_MODEL_INTERFACE(ADC)
{
	// ---- 输入端口 ----
	ADD_MODEL_INPUT(A_in);

	// ---- 输出端口 ----
	ADD_MODEL_OUTPUT(A_out);
	ADD_MODEL_OUTPUT(D_I);
	ADD_MODEL_OUTPUT(D_Q);

	// ---- 通用参数 ----
	{
		auto p = ADD_MODEL_PARAMETER(NBits);
		p.SetName("NBits");
		p.SetDescription("Number of bits");
	}
	{
		auto p = ADD_MODEL_PARAMETER(VRef);
		p.SetName("VRef");
		p.SetDescription("Reference voltage, -VRef <= input <= VRef");
		p.SetUnit(SystemVueModelBuilder::Units::VOLTAGE);
	}
	{
		auto p = ADD_MODEL_PARAMETER(SR);
		p.SetName("SR");
		p.SetDescription("Input sample rate (Hz)");
		p.SetUnit(SystemVueModelBuilder::Units::FREQUENCY);
	}
	{
		auto p = ADD_MODEL_PARAMETER(CenterFreq);
		p.SetName("CenterFreq");
		p.SetDescription("Spectral center frequency for analog input");
		p.SetUnit(SystemVueModelBuilder::Units::FREQUENCY);
	}

	// ---- OutputDigitalFormat（枚举）----
	{
		auto p = model.AddParamEnum<OutFmtType>(OutputDigitalFormat, "OutputDigitalFormat", "OutFmtType");
		p.SetName("OutputDigitalFormat");
		p.SetDescription("Output digital format: Offset binary, Twos-complement");
		p.AddEnumeration("Offset binary", OffsetBinary);
		p.AddEnumeration("Twos-complement", TwosComplement);
	}

	// ---- ConversionType（枚举）----
	{
		auto p = model.AddParamEnum<ConvType>(ConversionType, "ConversionType", "ConvType");
		p.SetName("ConversionType");
		p.SetDescription("Type of input conversion: Clocked, Downsampled");
		p.AddEnumeration("Clocked", Clocked);
		p.AddEnumeration("Downsampled", Downsampled);
	}

	// ---- 时钟设置（仅 Clocked 模式可见）----
	{
		auto p = ADD_MODEL_PARAMETER(Clock);
		p.SetName("Clock");
		p.SetDescription("Internal cosine clock frequency");
		p.SetUnit(SystemVueModelBuilder::Units::FREQUENCY);
		p.SetHideCondition("ConversionType ~= 0");
	}
	{
		auto p = ADD_MODEL_PARAMETER(Phase);
		p.SetName("Phase");
		p.SetDescription("Internal clock phase");
		p.SetUnit(SystemVueModelBuilder::Units::ANGLE);
		p.SetHideCondition("ConversionType ~= 0");
	}

	// ---- InterpMethod（枚举）----
	{
		auto p = model.AddParamEnum<InterpType>(InterpMethod, "InterpMethod", "InterpType");
		p.SetName("InterpMethod");
		p.SetDescription("Interpolation method: Linear, PCHIP, Spline");
		p.AddEnumeration("Linear", Linear);
		p.AddEnumeration("PCHIP", PCHIP);
		p.AddEnumeration("Spline", Spline);
	}

	// ---- 降采样参数（仅 Downsampled 模式可见）----
	{
		auto p = ADD_MODEL_PARAMETER(DownsampleFactor);
		p.SetName("DownsampleFactor");
		p.SetDescription("Downsampling ratio");
		p.SetHideCondition("ConversionType ~= 1");
	}
	{
		auto p = ADD_MODEL_PARAMETER(DownsamplePhase);
		p.SetName("DownsamplePhase");
		p.SetDescription("Downsampling phase");
		p.SetHideCondition("ConversionType ~= 1");
	}
	{
		auto p = model.AddParamEnum<ZohMappingType>(ZohMode, "ZohMode", "ZohMappingType");
		p.SetName("ZohMode");
		p.SetDescription("ZOH edge mapping: Discretize, Integer");
		p.SetHideCondition("ConversionType ~= 1");
		p.AddEnumeration("Discretize", Discretize);
		p.AddEnumeration("Integer", Integer);
	}

	// ---- AntiAliasingFilter + ExcessBW（仅 Downsampled 模式可见）----
	{
		auto p = model.AddParamEnum<AntiAliasingMode>(AntiAliasingFilter, "AntiAliasingFilter", "AntiAliasingMode");
		p.SetName("AntiAliasingFilter");
		p.SetDescription("Turn off/on anti-aliasing filter before downsampling: OFF, ON");
		p.SetHideCondition("ConversionType ~= 1");
		p.AddEnumeration("OFF", AA_OFF);
		p.AddEnumeration("ON", AA_ON);
	}
	{
		auto p = ADD_MODEL_PARAMETER(ExcessBW);
		p.SetName("ExcessBW");
		p.SetDescription("Excess bandwidth of raised cosine anti-aliasing filter");
		p.SetHideCondition("AntiAliasingFilter ~= 1");
	}

	return true;
}
#endif

// ===================== 构造函数 =====================

ADC::ADC()
{
	NBits = 12;
	VRef = 1.0;
	SR = 2400e6;
	CenterFreq = 0.0;
	Clock = 400e6;
	Phase = 0.0;
	DownsampleFactor = 1;
	DownsamplePhase = 0;
	ExcessBW = 0.5;

	ConversionType = Clocked;
	InterpMethod = Spline;
	OutputDigitalFormat = TwosComplement;
	ZohMode = Integer;
	AntiAliasingFilter = AA_ON;

	ResetStreamingState();
}

void ADC::ResetStreamingState()
{
	m_total_offset = 0;
	m_input_history.clear();

	int nbits = static_cast<int>(NBits);
	int mid_code = (nbits > 0) ? (1 << (nbits - 1)) : 0;
	m_last_DI = (OutputDigitalFormat == TwosComplement) ? 0 : mid_code;
	m_last_DQ = (OutputDigitalFormat == TwosComplement) ? 0 : mid_code;
}

// ===================== Setup =====================

bool ADC::Setup()
{
	if (SR <= 0)
	{
		POST_ERROR("SR must be greater than 0.");
		return false;
	}
	// 设置输出速率，进行批处理，每次处理100000个数据
	A_in.SetRate(100000);
	A_out.SetRate(100000);
	D_I.SetRate(100000);
	D_Q.SetRate(100000);

	// 设置时域采样率
	A_in.SetSampleRate(SR);
	A_out.SetSampleRate(SR);

	// 初始化/重置跨块状态
	ResetStreamingState();

	return true;
}

// ===================== PropagateCharacterizationFrequency =====================

ERESULT ADC::PropagateCharacterizationFrequency()
{
	if (CenterFreq > 0.0)
		A_out.SetCharacterizationFrequency(CenterFreq);
	else if (A_in.IsConnected())
		A_out.SetCharacterizationFrequency(A_in.GetCharacterizationFrequency());
	return NOERROR_;
}

// ===================== Run =====================

bool ADC::Run()
{
	if (!A_in.IsConnected())
		return true;

	size_t N = A_in.GetSize();
	if (N < 2)
		return true;

	// ---- 读取输入 ----
	std::vector<std::complex<double>> x_curr(N);
	double Ts = (SR > 0.0) ? (1.0 / SR) : 1e-9;
	for (size_t i = 0; i < N; ++i)
		x_curr[i] = A_in[i].complex();

	size_t hist = m_input_history.size();
	std::vector<std::complex<double>> x_in;
	x_in.reserve(hist + N);
	x_in.insert(x_in.end(), m_input_history.begin(), m_input_history.end());
	x_in.insert(x_in.end(), x_curr.begin(), x_curr.end());

	// ---- 构造参数 ----
	AdcParams params;
	params.n_bits = static_cast<int>(NBits);
	params.v_ref = VRef;
	params.sr = SR;
	params.center_freq = CenterFreq;
	params.clock = Clock;
	params.phase = Phase;
	params.conversion_type = ConversionType;
	params.interp_method = InterpMethod;
	params.output_digital_format = OutputDigitalFormat;
	params.zoh_mode = ZohMode;
	params.downsample_factor = static_cast<int>(DownsampleFactor);
	params.downsample_phase = static_cast<int>(DownsamplePhase);
	params.anti_aliasing_filter = AntiAliasingFilter;
	params.excess_bw = ExcessBW;
	params.init_DI = m_last_DI;
	params.init_DQ = m_last_DQ;

	size_t proc_offset = (m_total_offset >= hist) ? (m_total_offset - hist) : 0;
	double t_start = static_cast<double>(proc_offset) * Ts;

	// ---- 执行 ----
	AdcModel1 model;
	AdcOutput out = model.Process(x_in, Ts, t_start, params);

	// ---- 记录状态 ----
	size_t copy_start = hist;
	size_t available = (out.a_out.size() > copy_start) ? (out.a_out.size() - copy_start) : 0;
	size_t n_out = std::min(available, N);
	if (n_out > 0)
	{
		size_t last = copy_start + n_out - 1;
		m_last_DI = out.d_i[last];
		m_last_DQ = out.d_q[last];
	}
	m_total_offset += n_out;

	// ---- 写入输出 ----
	n_out = std::min(n_out, (size_t)A_out.GetSize());
	n_out = std::min(n_out, (size_t)D_I.GetSize());
	n_out = std::min(n_out, (size_t)D_Q.GetSize());
	for (size_t i = 0; i < n_out; ++i)
		A_out[i] = out.a_out[copy_start + i];
	D_I.CopyFrom(0, out.d_i.data() + copy_start, n_out);
	D_Q.CopyFrom(0, out.d_q.data() + copy_start, n_out);

	const size_t keep_history = 16;
	size_t keep = std::min(keep_history, x_in.size());
	m_input_history.assign(x_in.end() - keep, x_in.end());

	return true;
}
