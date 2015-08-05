#pragma once
#include "ISimulator.h"
#include "../Multiagent/MultiagentNE.h"
#include "../Multiagent/IMultiagentSystem.h"
#include "../Math/easymath.h"
#include <limits>
#include <sstream>
#include <cfloat>

using namespace std;
using namespace easymath;

class ISimNEParameters{
public:
	ISimNEParameters(void){};
	~ISimNEParameters(void){};
	static const int n_epochs = 100;
	static const int n_runs = 1;
	static const int n_trials = 1;
};

class SimNE: public ISimulator{
public:
	SimNE(IDomainStateful* domain);
	SimNE(IDomainStateful* domain, MultiagentNE* MAS);
	~SimNE(void);
	virtual void runExperiment();
	virtual void epoch(int ep);
	ISimNEParameters* sim_params;
	virtual matrix2d getActions(); // Gets actions based on current state: OVERLOAD FOR TYPES
};
