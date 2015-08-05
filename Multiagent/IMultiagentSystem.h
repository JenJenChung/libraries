#pragma once

// STL includes
#include <vector>

// Library includes
#include "../SingleAgent/IAgent.h"


typedef std::vector<std::vector<double> > matrix2d;
typedef std::vector<double> matrix1d;

class IMultiagentSystem
{
	// Agent container

public:
	IMultiagentSystem(void){}
	virtual ~IMultiagentSystem(void){}

	std::vector<IAgent*> agents; // Set of agents in the system (set externally)

	matrix2d getActions(matrix2d S){
	if (!S.size()){
		printf("Zero state size!");
		system("pause");
	}
	matrix2d A(agents.size());
	// get all actions, given a list of states
	for (int i=0; i<agents.size(); i++){
		A[i] = agents[i]->getAction(S[i]);
	}
	return A;
	
	}
	void updatePolicyValues(matrix1d R){
	for (int i=0; i<agents.size(); i++){
		agents[i]->updatePolicyValues(R[i]);
	}
	}
};
