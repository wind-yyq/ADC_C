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
	SR = 24e6;
	CenterFreq = 0.0;
	Clock = 10e6;
	Phase = 0.0;
	DownsampleFactor = 1;
	DownsamplePhase = 0;
	ExcessBW = 0.5;

	ConversionType = Clocked;
	InterpMethod = Spline;
	OutputDigitalFormat = TwosComplement;
	ZohMode = Integer;
	AntiAliasingFilter = AA_ON;
}

// ===================== Setup =====================

bool ADC::Setup()
{
	if (SR <= 0)
	{
		POST_ERROR("SR must be greater than 0.");
		return false;
	}

	A_in.SetRate(10000);
	A_out.SetRate(10000);
	D_I.SetRate(10000);
	D_Q.SetRate(10000);

	return true;
}

// ===================== PropagateCharacterizationFrequency =====================

ERESULT ADC::PropagateCharacterizationFrequency()
{
	// 输入包络特征频率直通到输出，或使用 CenterFreq 覆盖
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
	size_t No = std::min({N, (size_t)D_I.GetSize(), (size_t)D_Q.GetSize()});
	if (N < 2 || No < 1)
		return true;

	// ---- 从包络环形缓冲区读取输入复包络 ----
	std::vector<std::complex<double>> x_in(No);
	double Ts = (SR > 0.0) ? (1.0 / SR) : 1e-9;
	for (size_t i = 0; i < No; ++i)
		x_in[i] = A_in[i].complex();

	// ---- 构建参数结构体 ----
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

	// ---- 调用核心算法 ----
	AdcModel1 model;
	AdcOutput out = model.Process(x_in, Ts, params);

	// ---- 写入输出环形缓冲区 ----
	for (size_t i = 0; i < No; ++i)
		A_out[i] = out.a_out[i];   // EnvelopeSignal 必须逐元素赋值（防 Q 丢失）
	D_I.CopyFrom(0, out.d_i.data(), No);
	D_Q.CopyFrom(0, out.d_q.data(), No);

	return true;
}
