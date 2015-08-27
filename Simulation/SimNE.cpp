#include "SimNE.h"

SimNE::SimNE(IDomainStateful* domain):
	ISimulator(domain, new MultiagentNE(domain->n_agents, new NeuroEvoParameters(domain->n_state_elements,domain->n_control_elements)))
{
	sim_params = new ISimNEParameters();
}

// FOR DEBUGGING
SimNE::SimNE(IDomainStateful* domain, MultiagentNE* MAS):
	ISimulator(domain, MAS)
{
	sim_params = new ISimNEParameters();
}

SimNE::~SimNE(void)
{
	delete sim_params;
	delete ((MultiagentNE*)MAS)->NE_params;
	delete MAS;
}

void SimNE::runExperiment(){
	for (int ep=0; ep<sim_params->n_epochs; ep++){
		printf("Epoch %i\n",ep);
		//printf(".");
		this->epoch(ep);
	}
}

void SimNE::epoch(int ep){
	((MultiagentNE*)MAS)->generateNewMembers();
	double best_run = -DBL_MAX;
	double best_run_performance = -DBL_MAX;

	int n=0; // neural net number (for output file name)
	clock_t tref = clock();

	do{
		matrix2d Rtrials; // Trial average reward
		for (int tt=0; tt<sim_params->n_trials; tt++){
			for (int s=0; s<domain->n_steps; s++){
//				printf("Step %i.0\n",s);
				matrix2d A = this->getActions(); // must be called by 'this' in order to access potential child class overload
//				printf("Step %i.1\n",s);
				domain->simulateStep(A);
//				printf("Step %i.2\n",s);
				domain->logStep(s);
//				printf("Step %i.3\n",s);
			}
			int t= clock();
			printf("t_simulate=%f\n",float(t-tref)/CLOCKS_PER_SEC);
			tref=t;

			matrix1d R = domain->getRewards();
			matrix1d perf = domain->getPerformance();

			double avg_G = mean(R);
			double avg_perf = mean(perf);

			// note: things here get specific to the utm domain
			if (avg_G>best_run) {
				best_run = avg_G;
				best_run_performance = avg_perf; // average of the performance metrics
				if (ep==0){
					domain->exportLog("stat_results/conflict_map-0-", ep); // blatant abuse of exportlog
				}
				if (ep==99){
					domain->exportLog("stat_results/conflict_map-99-",ep);
				}
			}
			
			printf("NN#%i, %f\n",n, best_run);
			//printf(".");
			ostringstream epi,ni,ti;
			epi << ep;
			ni << n;
			ti << t;
			
			Rtrials.push_back(R);

			domain->reset();
		}
		MAS->updatePolicyValues(mean2(Rtrials)); // based on the trials...
		
		n++;
		
	} while (((MultiagentNE*)MAS)->setNextPopMembers()); // this also takes care of reset functions
	((MultiagentNE*)MAS)->selectSurvivors();


	reward_log.push_back(best_run);
	metric_log.push_back(best_run_performance);
}

matrix2d SimNE::getActions(){
	matrix2d S = domain->getStates();
	return MAS->getActions(S);
}
