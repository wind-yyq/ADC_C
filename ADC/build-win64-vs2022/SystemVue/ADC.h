#pragma once
#include "SystemVue/ModelBuilder.h"

class ADC : public SystemVueModelBuilder::DFModel
{
public:
	// This Macro is required for all classes derived from DFModel
	DECLARE_MODEL_INTERFACE( ADC );

	// Constructor to initialize parameters
	ADC();
	
	//-------- Function Overloads --------
	virtual bool	Run();

	// Ports
	SystemVueModelBuilder::CircularBuffer< double > input, output;
	
	// Parameter
	double Gain;

};
