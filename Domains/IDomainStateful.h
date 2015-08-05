#pragma once

#include <vector>
#include <string>

typedef std::vector<double> matrix1d;
typedef std::vector<std::vector<double> > matrix2d;
typedef std::vector<std::vector<std::vector<double> > > matrix3d;

class IDomainStateful
{
public:
	IDomainStateful(void);
	~IDomainStateful(void);

	virtual matrix2d getStates()=0; // Returns the state vector for the set of agents, [AGENTID][STATEELEMENT]
	virtual matrix3d getTypeStates()=0; // [AGENTID][TYPEID][STATEELEMENT]
	virtual matrix1d getRewards()=0; // Returns the reward vector for a set of agents [AGENTID]
	virtual matrix1d getPerformance()=0; // returns the performance vector for a set of agents
	virtual void simulateStep(matrix2d agent_actions)=0; // Simulates a step
	virtual void reset()=0;
	virtual void logStep(int step)=0;
	virtual void exportLog(std::string fid, double G)=0;

	// constants
	bool type_blind;
	int n_state_elements;
	int n_control_elements;
	int n_steps;
	int n_agents;
	int n_types;
	std::vector<int> fixed_types; // types fixed in the domain (MAYBE NOT HAVE THESE HERE?)

};

