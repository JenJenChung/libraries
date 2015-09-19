#pragma once

// STL includes
#include <utility>
#include <cmath>
#include <fstream>
#include <sstream>

// Library includes
#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../../FileIO/easyio/easyio.h"
#include "../../Planning/RAGS.h"
#include "../../Math/Matrix.h"



#define WORLD_SIZE 100.0


using namespace std;
using namespace easymath;
using namespace Numeric_lib;

typedef Matrix<bool,2> barrier_grid;
typedef Matrix<int,2> ID_grid;

class UAV{
	/*
	This class is for moving UAVs in the airspace. They interact with the 
	environment through planning. Planning is done through RAGS. 
	*/
public:
	enum UAVType{SLOW, FAST, NTYPES=2}; // original: const enum UAVType{SLOW, FAST, NTYPES=5};
	//const enum UAVType{SLOW,NTYPES};

	// JEN: each UAV to have its own RAGS object
	UAV(XY start_loc, XY goal_loc, UAVType type, vector<XY> &locations, vector<RAGS::edge> &edge_array, matrix2d weights) ;
	// END

	~UAV(){
		delete itsRAGS;
		itsRAGS = 0 ;
	};

	int getDirection(); // gets the cardinal direction of the UAV
	// ABSTRACTION MODE PLANNING ONLY, will need to upgrade to high fidelity simulation in future
	// calls RAGS to search next best connection
	void pathPlan(bool abstraction_mode, vector<double> &connection_time, vector<double> &weights) ;

	int ID;
	UAVType type_ID;
	double speed; // connected to type_ID
	easymath::XY loc ; // current sector location
	easymath::XY next_loc ; // next sector location
	easymath::XY end_loc ; // destination sector location
	RAGS *itsRAGS ; // path planning management
	// ABSTRACTION MODE; TIME UNTIL SWITCH
	int t;
};

class Fix{
public:
	Fix(XY loc, int ID, bool deterministic);
	~Fix(){};

	std::list<UAV*> generateTraffic(vector<Fix>* fixes, barrier_grid* obstacle_map, vector<XY> &locations, vector<RAGS::edge> &edge_array, matrix3d &weights);
	bool atDestinationFix(const UAV &u);
	int ID;
	
	bool is_deterministic;
	XY loc;
	const double p_gen;
	const double dist_thresh;
	static const int gen_frequency=10;
};

class Sector{
public:
	// An area of space that contains some fixes
	Sector(XY xy);
	Sector(){}; // default constructor
	~Sector(){};
	XY xy; // sector center
};

class ATFMSectorDomain: public IDomainStateful
{
public:
	enum TYPECap{TYPE1=2, TYPE2=5, TYPE3=3, TYPE4=1, TYPE5=4, TYPE6=10};
	ATFMSectorDomain(bool deterministic=false);
	~ATFMSectorDomain(void);

	// Base function overloads
	matrix1d getRewards();
	matrix1d getPerformance();
	matrix2d getStates();
	matrix3d getTypeStates();

	bool is_deterministic; // the simulation is deterministic (for testing learning)
	bool abstraction_mode; // in this mode, there is no low-level planning, and a simple network is used

	unsigned int getSector(XY p);

	vector<Sector>* sectors;
	list<UAV*>* UAVs; // this is in a list because it has to be modified often. Never tie an ID/index to a UAV
	vector<Fix>* fixes;

	vector< vector<int> > direction_map; // direction (cardinal) needed to travel to go from [node1][node2]

	void simulateStep(matrix2d agent_actions);

	void setCostMaps(matrix2d agent_actions);
	void getNewUAVTraffic();
	void absorbUAVTraffic();
	void getPathPlans(); // note: when is this event?
	void getPathPlans(std::list<UAV> &new_UAVs);

	void reset();
	void logStep(int step);
	void exportLog(std::string fid, double G);

	void load_variable(Matrix<int,2> &var, std::string filename, std::string separator = STRING_UNINITIALIZED){
		// must be above threshold to be counted as a boolean
		string_matrix2d f = FileManip::read(filename, separator);
		
		for (unsigned i=0; i<f.size(); i++){
			for (unsigned j=0; j<f[i].size(); j++){
				var(i,j) = atoi(f[i][j].c_str());
			}
		}
	}

	void load_variable(Matrix<bool,2> &var, std::string filename, double thresh, std::string separator = STRING_UNINITIALIZED){
		// must be above threshold to be counted as a boolean
		string_matrix2d f = FileManip::read(filename, separator);
		
		for (unsigned i=0; i<f.size(); i++){
			for (unsigned j=0; j<f[i].size(); j++){
				if (atof(f[i][j].c_str())<=thresh){
					var(i,j) = false;
				} else {
					var(i,j) = true;
				}
			}
		}
	}

	void load_variable(std::vector<std::vector<bool> >* var, std::string filename, double thresh, std::string separator = STRING_UNINITIALIZED){
		// must be above threshold to be counted as a boolean
		string_matrix2d f = FileManip::read(filename, separator);
		*var = std::vector<std::vector<bool> >(f.size());

		for (unsigned i=0; i<f.size(); i++){
			var->at(i) = std::vector<bool>(f[i].size());
			for (unsigned j=0; j<f[i].size(); j++){
				if (atof(f[i][j].c_str())<=thresh){
					var->at(i)[j] = false;
				} else {
					var->at(i)[j] = true;
				}
			}
		}
	}

	// Conflict detection/logging
	void detectConflicts();
	int conflict_count;
	Matrix<int,2> *conflict_count_map; // this counts the number of conflicts in each grid (reset() clears this)

	//backend
	Matrix<int,2> * membership_map; // technically this should be an int matrix. fix later
	Matrix<bool,2> * obstacle_map; // pass these to uavs later to determine where the obstacles are

	// For RAGS
	vector<XY> agent_locs;
	vector<RAGS::edge> edges ; // list of edges
	matrix1d connection_time ; // list of connection times for each edge
	matrix2d agent_capacity ; // capacity of each agent for each type of UAV
	matrix3d weights ; // [mean/var][type][edge]
	list< vector< vector<double> > > weights_history ; // [timestep][type][edge] list first dimension for sliding window use

	//map<list<AStar_easy::vertex>, AStar_easy*> astar_lowlevel;
	map<int,pair<int,int> > sector_dir_map; // maps index of edge to (sector next, direction of travel)

	matrix2d overcap;
	matrix3d counterOvercap;
	matrix1d globalPerformance ;

	void count_overcap(){
		if(!overcap.size()){
			overcap = matrix2d(n_agents);

			for (int i=0; i<overcap.size(); i++){
				overcap[i] = matrix1d(UAV::NTYPES,0.0) ;
			}
		}
		// Count total UAVs on each connection
		matrix2d current_cap = agent_capacity ;
		for (list<UAV*>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
			current_cap[getSector((*u)->loc)][(*u)->type_ID]-=1.0;
			if (current_cap[getSector((*u)->loc)][(*u)->type_ID] < 0.0)
				overcap[getSector((*u)->loc)][(*u)->type_ID]-=1.0 ;
		}
	}
	
	void CounterFactual(){
		if(!counterOvercap.size())
			counterOvercap = matrix3d(n_agents,matrix2d(n_agents,matrix1d(UAV::NTYPES,0.0)));
		
		// loop through all UAVs, any UAVs at agent_loc are reassigned to their next_loc
		int i = 0 ;
		for(Fix f: *fixes){
			// Count total UAVs on each connection
			matrix2d current_cap = agent_capacity ;
			for (list<UAV*>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
				if ((*u)->loc == f.loc){
					current_cap[getSector((*u)->next_loc)][(*u)->type_ID]-=1.0;
					if (current_cap[getSector((*u)->next_loc)][(*u)->type_ID] < 0.0)
						counterOvercap[i][getSector((*u)->next_loc)][(*u)->type_ID]-=1.0 ;
				}
				else{
					current_cap[getSector((*u)->loc)][(*u)->type_ID]-=1.0;
					if (current_cap[getSector((*u)->loc)][(*u)->type_ID] < 0.0)
						counterOvercap[i][getSector((*u)->loc)][(*u)->type_ID]-=1.0 ;
				}
			}
			i++ ;
		}
	}
};

