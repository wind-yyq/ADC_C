#pragma once
#include "SystemVue/ModelBuilder.h"
#include "SystemVue/TimedDFModel.h"
#include "SystemVue/EnvelopeSignal.h"
#include "ADC_model1.h"

class ADC : public SystemVueModelBuilder::TimedDFModel
{
public:
	DECLARE_MODEL_INTERFACE(ADC);
	ADC();
	virtual bool Setup();
	virtual bool Run();
	virtual ERESULT PropagateCharacterizationFrequency();

	// ---- 输入端口 ----
	SystemVueModelBuilder::EnvelopeCircularBuffer A_in;

	// ---- 输出端口 ----
	SystemVueModelBuilder::EnvelopeCircularBuffer A_out;
	SystemVueModelBuilder::CircularBuffer<int> D_I, D_Q;

	// ---- 参数 ----
	double      NBits;
	double      VRef;
	double      SR;
	double      CenterFreq;
	double      Clock;
	double      Phase;
	double      DownsampleFactor;
	double      DownsamplePhase;
	double      ExcessBW;

	ConvType    ConversionType;
	InterpType  InterpMethod;
	OutFmtType  OutputDigitalFormat;
	ZohMappingType ZohMode;

	AntiAliasingMode AntiAliasingFilter;

private:
	void ResetStreamingState();

	size_t   m_total_offset;
	int      m_last_DI;
	int      m_last_DQ;
	std::vector<std::complex<double>> m_input_history;
};
