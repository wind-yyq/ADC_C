#include "ADC.h"

#ifndef SV_CODE_GEN
DEFINE_MODEL_INTERFACE ( ADC )
{	
	ADD_MODEL_INPUT( input );
	ADD_MODEL_OUTPUT( output );
	ADD_MODEL_PARAMETER( Gain );
	return true;
}
#endif

ADC::ADC()
{
	Gain = 0;
}

//-----------------------------------------------------------------------------------
//	Run
//		Here we do the math
//-----------------------------------------------------------------------------------
bool ADC::Run()
{
	output[0] = Gain * input[0];
	return true;
}
