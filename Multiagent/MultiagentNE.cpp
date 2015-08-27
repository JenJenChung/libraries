#include "MultiagentNE.h"

MultiagentNE::MultiagentNE(void){};

MultiagentNE::MultiagentNE(int n_agents, NeuroEvoParameters* NE_params):
	NE_params(NE_params)
{
	for (int i=0; i<n_agents; i++){
		agents.push_back(new NeuroEvo(NE_params));
	}
}

MultiagentNE::~MultiagentNE(void){
	for (int i=0; i<agents.size(); i++){
		delete agents[i];
	}
};

void MultiagentNE::generateNewMembers(){
	// Generate new population members
	for (int i=0; i<agents.size(); i++){
		((NeuroEvo*)agents[i])->generateNewMembers();
	}
}

void MultiagentNE::selectSurvivors(){
	// Specific to Evo: select survivors
	for (int i=0; i<agents.size(); i++){
		((NeuroEvo*)agents[i])->selectSurvivors();
	}
}

bool MultiagentNE::setNextPopMembers(){
	// Kind of hacky; select the next member and return true if not at the end
	// Specific to Evo

	std::vector<bool> is_another_member(agents.size(),false);
	for (int i=0; i<agents.size(); i++){
		is_another_member[i] = ((NeuroEvo*)agents[i])->selectNewMember();
	}
	for (int i=0; i<is_another_member.size(); i++){
		if (!is_another_member[i]){
			return false;
		}
	}
	return true;
}
