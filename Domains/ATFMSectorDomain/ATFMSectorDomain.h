#pragma once

// STL includes
#include <utility>

// Library includes
#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../../FileIO/easyio/easyio.h"
#include "../../Planning/AStar_easy.h"
#include "../../Planning/AStar_grid.h"
#include "../../Math/Matrix.h"



#define WORLD_SIZE 100.0


using namespace std;
using namespace easymath;

typedef Matrix<bool,2> barrier_grid;
typedef Matrix<int,2> ID_grid;
typedef std::map<int,std::map<int,AStar_grid*> > grid_lookup;

class Sector;

class UAV{
	/*
	This class is for moving UAVs in the airspace. They interact with the 
	environment through planning. Planning is done through boost. 
	*/
public:
	enum UAVType{SLOW, FAST, NTYPES=5}; // original: const enum UAVType{SLOW, FAST, NTYPES=5};
	//const enum UAVType{SLOW,NTYPES};

	UAV(easymath::XY start_loc, easymath::XY end_loc,
		std::vector<std::vector<XY> > *pathTraces, UAVType t);

	~UAV(){};

	int getDirection(); // gets the cardinal direction of the UAV
	void moveTowardNextWaypoint(); // takes a time increment to move over
	void pathPlan(AStar_easy* Astar_highlevel, grid_lookup &m2astar, barrier_grid*obstacle_map,
		ID_grid* membership_map, vector<Sector>* sectors, bool abstraction_mode, int connection_time[15][15]);

	int ID;
	UAVType type_ID;
	double speed; // connected to type_ID
	easymath::XY loc;
	easymath::XY end_loc;
	std::queue<easymath::XY> target_waypoints; // target waypoints, low-level
	std::vector<std::vector<XY> > *pathTraces; // takes a pointer to the pathtrace for logging
	list<AStar_easy::vertex> high_path_prev; // saves the high level path

	// ABSTRACTION MODE; TIME UNTIL SWITCH
	int t;
};

class Fix{
public:
	Fix(XY loc, int ID, bool deterministic);
	~Fix(){};

	std::list<UAV> generateTraffic(std::vector<Fix>* fixes, barrier_grid* obstacle_map,std::vector<std::vector<XY> > *pathTraces);
	void absorbTraffic(std::list<UAV>* UAVs);
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
	ATFMSectorDomain(bool deterministic=false);
	~ATFMSectorDomain(void);

	// Base function overloads
	matrix1d getRewards();
	matrix1d getPerformance();
	matrix2d getStates();
	matrix3d getTypeStates();

	bool is_deterministic; // the simulation is deterministic (for testing learning)
	bool abstraction_mode; // in this mode, there is no low-level planning, and a simple network is used
	int connection_time[15][15]; // time to travel from first to second agent == HARDCODED FOR 15 AGENTS
	int connection_capacity[15][15][UAV::NTYPES]; // capacity for each type of UAV

	unsigned int getSector(easymath::XY p);

	std::vector<Sector>* sectors;
	std::list<UAV>* UAVs; // this is in a list because it has to be modified often. Never tie an ID/index to a UAV
	std::vector<Fix>* fixes;

	std::vector<std::vector<int> > direction_map; // direction (cardinal) needed to travel to go from [node1][node2]

	void simulateStep(matrix2d agent_actions);
	void incrementUAVPath();

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
		
		for (int i=0; i<f.size(); i++){
			for (int j=0; j<f[i].size(); j++){
				var(i,j) = atoi(f[i][j].c_str());
			}
		}
	}

	void load_variable(Matrix<bool,2> &var, std::string filename, double thresh, std::string separator = STRING_UNINITIALIZED){
		// must be above threshold to be counted as a boolean
		string_matrix2d f = FileManip::read(filename, separator);
		
		for (int i=0; i<f.size(); i++){
			for (int j=0; j<f[i].size(); j++){
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

		for (int i=0; i<f.size(); i++){
			var->at(i) = std::vector<bool>(f[i].size());
			for (int j=0; j<f[i].size(); j++){
				if (atof(f[i][j].c_str())<=thresh){
					var->at(i)[j] = false;
				} else {
					var->at(i)[j] = true;
				}
			}
		}
	}


	// PATH SNAPSHOT OUTPUT
	void pathSnapShot(int snapnum);
	void pathTraceSnapshot();
	vector<vector<XY> > *pathTraces; // [UAV_ID][step]

	// Conflict detection/logging
	void detectConflicts();
	int conflict_count;
	Matrix<int,2> *conflict_count_map; // this counts the number of conflicts in each grid (reset() clears this)

	//backend
	Matrix<int,2> * membership_map; // technically this should be an int matrix. fix later
	Matrix<bool,2> * obstacle_map; // pass these to uavs later to determine where the obstacles are
	//matrix2d* connection_map;


	// for A* (boost)
	vector<XY> agent_locs;
	vector<AStar_easy::edge> edges;
	// vector<double> weights; // old
	matrix2d weights; // [type][connection]
	std::vector<AStar_easy*> Astar_highlevel;
	grid_lookup m2astar;
	//AStar_easy* Astar_highlevel; // old

	//map<list<AStar_easy::vertex>, AStar_easy*> astar_lowlevel;
	map<int,pair<int,int> > sector_dir_map; // maps index of edge to (sector next, direction of travel)


	void printMasks(){
		for (grid_lookup::iterator it=m2astar.begin(); it!=m2astar.end(); it++){
			for (map<int,AStar_grid*>::iterator inner = it->second.begin(); inner!=it->second.end(); inner++){
				inner->second->m.printMap("masks/", it->first, inner->first);
			}
		}
	}


	/*template <class T>
	void initialize(vector<T>* p_vector, string file_name){
		
	}*/

	matrix2d overcap;

	void count_overcap(){
		if(!overcap.size()){
			overcap = matrix2d(n_agents);

			for (int i=0; i<overcap.size(); i++){
				overcap[i] = matrix1d(UAV::NTYPES,0.0); // starts at 'capacity'
			}
		}

		for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
			overcap[getSector(u->loc)][u->type_ID]-=1.0;
			//L[getSector(u->loc)] =  
		}
	}
};

