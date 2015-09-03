#include "ATFMSectorDomainRAGS.h"

using namespace std;
using namespace easymath;

UAV::UAV(XY start_loc, XY goal_loc, UAVType type, vector<XY> &locations, vector<RAGS::edge> &edge_array, matrix2d weights):
	next_loc(start_loc), end_loc(goal_loc), type_ID(type)
{
	t = 0 ;
	itsRAGS = new RAGS(locations, edge_array, weights) ;
	switch(t){
	case SLOW:
		speed = 1;
		break;
	case FAST:
		// substitute: high priority
		speed = 1;
		//speed = 2;
		break;
	default:{
		//printf("no speed found!!");
		speed = 1;
		//system("pause");
		break;
		}
	}

};

void UAV::pathPlan(bool abstraction_mode, vector<double> &connection_time, vector<double> &weights)
{
	if (abstraction_mode){
		if (t == 0){ // reached new sector
			loc = next_loc ;
			if (!(loc == end_loc)){ // have not reached destination
				next_loc = itsRAGS->SearchGraph(loc,end_loc,weights) ;
				int i = itsRAGS->GetEdgeIndex(loc,next_loc) ;
				t = connection_time[i] ;
			}
		}
		else
			t-- ;
	}
	else {
			printf("ERROR: Simulation currently only available in abstraction mode. Exiting.\n") ;
			exit(1) ;
	}
}

int UAV::getDirection(){
	// Identifies whether traveling in one of four cardinal directions
	return cardinalDirection(next_loc-loc);
}

Sector::Sector(XY xy): xy(xy)
{
}

Fix::Fix(XY loc, int ID_set, bool deterministic): 
	is_deterministic(deterministic), ID(ID_set), loc(loc), 
	//p_gen(0.05) // change to 1.0 if traffic controlled elsewhere
	p_gen(int(is_deterministic)*(1.0-0.05)+0.05), // modifies to depend on if deterministic
	dist_thresh(2.0)
{
}

bool Fix::atDestinationFix(const UAV &u){
	return u.end_loc == loc;
}

list<UAV*> Fix::generateTraffic(vector<Fix>* allFixes, barrier_grid* obstacle_map, vector<XY> &locations, vector<RAGS::edge> &edge_array, matrix3d &weights){
//list<UAV> Fix::generateTraffic(vector<Fix>* allFixes, barrier_grid* obstacle_map, vector<XY> &locations, vector<RAGS::edge> &edge_array, matrix3d &weights){
	static int calls = 0;
	// Creates a new UAV in the world
	std::list<UAV*> newTraffic;
	
	// CONSTANT TRAFFIC FLOW METHOD
	double coin = COIN_FLOOR0;
	if (coin<p_gen){
		XY end_loc;
		if (ID==0){
			end_loc = allFixes->back().loc;
		} else {
			end_loc = allFixes->at(ID-1).loc; // go to previous
		}
		UAV::UAVType type_id_set = UAV::UAVType(calls%int(UAV::UAVType::NTYPES)); // EVEN TYPE NUMBER
		UAV * newUAV = new UAV(loc,end_loc,type_id_set,locations,edge_array,weights[type_id_set]) ;
		newTraffic.push_back(newUAV);
//		newTraffic.push_back(*newUAV);
	}
	
	/* VARIABLE TRAFFIC METHOD
	double coin = COIN_FLOOR0;
	if (coin<p_gen){
		XY end_loc;
		do {
			end_loc = allFixes->at(COIN_FLOOR0*allFixes->size()).loc;
		} while (end_loc==loc);
		UAV::UAVType type_id_set = UAV::UAVType(calls%int(UAV::UAVType::NTYPES)); // EVEN TYPE NUMBER
		newTraffic.push_back(UAV(loc,end_loc,type_id_set,search));
	}*/

	calls++;
	return newTraffic;
}

ATFMSectorDomain::ATFMSectorDomain(bool deterministic):
	is_deterministic(deterministic), abstraction_mode(true) // hardcode for the abstraction...
{
	// Object creation
	sectors = new vector<Sector>();
	fixes = new vector<Fix>();
	UAVs = new list<UAV*>();

	// inherritance elements: constant
	//n_control_elements=4; // 4 outputs for sectors (cost in cardinal directions) (no types)
	n_control_elements=4*UAV::NTYPES;
	n_state_elements=4; // 4 state elements for sectors ( number of planes traveling in cardinal directions)
	n_steps=100; // steps of simulation time
	n_types=UAV::NTYPES;

	// Read in files for sector management
	obstacle_map = new barrier_grid(256,256);
	membership_map = new ID_grid(256,256); //HARDCODED
	load_variable(*membership_map,"agent_map/membership_map.csv");

	for (int i=0; i<membership_map->dim1(); i++){
		for (int j=0; j<membership_map->dim2(); j++){
			obstacle_map->at(i,j) = membership_map->at(i,j)<0;
		}
	}

	matrix2d agent_coords = FileManip::readDouble("agent_map/agent_map.csv");
	matrix2d connection_map = FileManip::readDouble("agent_map/connections.csv");

	matrix2d fix_locs = FileManip::readDouble("agent_map/fixes.csv");
	if (abstraction_mode) fix_locs = agent_coords; // if using an abstraction, have only the centers phsyically located

	// Add sectors
	agent_locs = vector<XY>(agent_coords.size()); // save this for later Astar recreation
	for (unsigned i=0; i<agent_coords.size(); i++){
		sectors->push_back(Sector(XY(agent_coords[i][0],agent_coords[i][1])));
		agent_locs[i] = sectors->back().xy;
	}
	
	// initialize fixes
	for (unsigned i=0; i<fix_locs.size(); i++){
		fixes->push_back(Fix(XY(fix_locs[i][0],fix_locs[i][1]),i,is_deterministic));
	}

	n_agents = sectors->size(); // number of agents dictated by read in file
	connection_capacity = matrix2d(n_agents,matrix1d(UAV::NTYPES,5.0)) ; // flat capacity across all types

	// Adjust the connection map to be the edges
	// preprocess boolean connection map
	// map edges to (sector ind, direction ind)
	for (unsigned i=0; i<connection_map.size(); i++){
		for (unsigned j=0; j<connection_map[i].size(); j++){
			if (connection_map[i][j] && i!=j){
				XY xyi = agent_locs[i];
				XY xyj = agent_locs[j];
				XY dx_dy = xyi-xyj;
				int xydir = cardinalDirection(dx_dy);
				int memj = membership_map->at(xyi); // costs applied to outgoing edges
				sector_dir_map[edges.size()] = make_pair(memj,xydir); // add at new index
				edges.push_back(RAGS::edge(i,j));
				
				// JEN: store connection times and capacities for abstraction mode
				if (abstraction_mode){
					if (n_agents!=15){
						printf("WRONG CONNECTION SIZE!");
						exit(1);
					}
//					connection_time.push_back(distance(agent_locs[i],agent_locs[j])); // use Euclidean instead of Manhattan
					XY diff = agent_locs[i]-agent_locs[j];
					connection_time.push_back(abs(diff.x)+abs(diff.y)); // use Manhattan distance
				}
			}
		}
	}

	// JEN: Initialise weights with variance, log to weights history
	// Set base edge costs as Euclidean distance + Euclidean distance * agent_action
	// initialise agent actions to 1.0 to give initial estimate of cost as 2*Euclidean distance
	// initialise variance to 0.1
	weights = matrix3d(2) ;
	matrix2d w_mean(UAV::NTYPES) ;
	matrix2d w_var(UAV::NTYPES) ;
	for (unsigned i=0; i<w_mean.size(); i++){
//		w_mean[i] = matrix1d(connection_time.size(),1.0) ;//******************************************
//		w_var[i] = matrix1d(connection_time.size(),0.1) ;//*******************************************
		w_mean[i] = DotMultiply(connection_time,2.0);//*************************************************
		w_var[i] = DotMultiply(connection_time,0.3);//**************************************************
}
	weights[0] = w_mean ;
	weights[1] = w_var ;
	weights_history.push_back(weights[0]) ;
	// END

	conflict_count = 0; // initialize with no conflicts
	
	conflict_count_map = new ID_grid(obstacle_map->dim1(), obstacle_map->dim2());
}

ATFMSectorDomain::~ATFMSectorDomain(void)
{
	delete UAVs;
	delete fixes;
	delete sectors;
	delete obstacle_map;
	delete membership_map;
	delete conflict_count_map;
}


vector<double> ATFMSectorDomain::getPerformance(){
	if (abstraction_mode){
		return getRewards();
	} else {
		return matrix1d(sectors->size(),-conflict_count);
	}
}

vector<double> ATFMSectorDomain::getRewards(){
	// LINEAR REWARD
	if (!abstraction_mode){
		return matrix1d(sectors->size(),-conflict_count); // linear reward


		// QUADRATIC REWARD
		/*int conflict_sum = 0;
		for (int i=0; i<conflict_count_map->size(); i++){
		for (int j=0; j<conflict_count_map->at(i).size(); j++){
		int c = conflict_count_map->at(i)[j];
		conflict_sum += c*c;
		}
		}
		return matrix1d(sectors->size(),-conflict_sum);*/
	} else {
		//int overcap[n_agents][UAV::NTYPES];

		double G=0;
		for (unsigned i=0; i<overcap.size(); i++){
			for (unsigned j=0; j<overcap[i].size(); j++){
				if (overcap[i][j] < 0.0){
					G -= abs(overcap[i][j]) ;	// use linear cost
//					G -= overcap[i][j]*overcap[i][j]; // worse with higher concentration of planes!*********
				}
				// clear overcap once used!
				overcap[i][j] = 0.0 ;
			}
		}
		return matrix1d(n_agents, G); // global reward
	}
}

matrix2d ATFMSectorDomain::getStates(){
	// States: delay assignments for UAVs that need routing
	matrix2d allStates(n_agents);
	for (int i=0; i<n_agents; i++){
		allStates[i] = matrix1d(n_state_elements,0.0); // reserves space
	}

	for (list<UAV*>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		allStates[getSector((*u)->loc)][(*u)->getDirection()]+=1.0; // Adds the UAV impact on the state
	}

	return allStates;
}

matrix3d ATFMSectorDomain::getTypeStates(){
	matrix3d allStates(n_agents);
	for (int i=0; i<n_agents; i++){
		allStates[i] = matrix2d(n_types);
		for (int j=0; j<n_types; j++){
			allStates[i][j] = matrix1d(n_state_elements,0.0);
		}
	}

	for (list<UAV*>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		int a = getSector((*u)->loc);
		int id = (*u)->type_ID;
		int dir = (*u)->getDirection();
		if (a<0){
			printf("UAV %i at location %f,%f is in an obstacle.!", (*u)->ID,(*u)->loc.x,(*u)->loc.y);
			system("pause");
		}
		allStates[a][id][dir]+=1.0;
	}

	return allStates;
}

unsigned int ATFMSectorDomain::getSector(easymath::XY p){
	// tests membership for sector, given a location
	return membership_map->at(p);
}

void ATFMSectorDomain::getPathPlans(){
	// sets own next waypoint
	for (list<UAV*>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		matrix2d w = weights_history.back() ;
		(*u)->pathPlan(abstraction_mode, connection_time, w[(*u)->type_ID]);
	}
}

void ATFMSectorDomain::simulateStep(matrix2d agent_actions){
	static int calls=0;
	setCostMaps(agent_actions);
	absorbUAVTraffic();
	if (calls%10==0)
		getNewUAVTraffic();
	calls++;
	getPathPlans();
	detectConflicts();
}

void ATFMSectorDomain::setCostMaps(vector<vector<double> > agent_actions){
	matrix2d w_val = weights_history.back() ;

	for (unsigned i=0; i<w_val[0].size(); i++){
		for (unsigned j=0; j<UAV::NTYPES; j++){
			unsigned s = sector_dir_map[i].first;
			unsigned d = j*(UAV::NTYPES-1) + sector_dir_map[i].second;
			
			if (s < 0 || s >= agent_actions.size())
				printf("s: %i", s) ;
			if (d < 0 || d >= agent_actions[s].size())
				printf("d: %i", d) ;
			
			// Scale agent actions against connection time
			w_val[j][i] = (1.0+agent_actions[s][d]) * connection_time[i] ;//****************************
//			w_val[j][i] = agent_actions[s][d] ;//*******************************************************
			
			if (w_val[j][i] < 0.0){
				printf("Negative weight at (j,i): (%i,%i), (s,d): (%i,%i)", j, i, s, d) ;
				printf("w_val[j][i]: %f, agent_actions[s][d]: %f", w_val[j][i], agent_actions[s][d]) ;
			}
		}
	}
	weights_history.push_back(w_val) ;
	
	// Keep most recent 100 weights
	if (weights_history.size() > 100)
		weights_history.pop_front() ;
	
	// Calculate new mean and variance for edge weights
	matrix1d w_t(weights_history.size(),0.0) ;
	for (int i = 0; i < UAV::NTYPES; i++){
		for (unsigned j = 0; j < edges.size(); j++){
			int k = 0 ;
			for (list<matrix2d>::iterator w = weights_history.begin(); w != weights_history.end(); w++){
				matrix2d wTemp = *w ;
				w_t[k++] = wTemp[i][j] ;
			}
			matrix1d w_mv = calc_mean_var(w_t) ;
			// Update weights matrix
			weights[0][i][j] = w_mv[0] ;
			weights[1][i][j] = w_mv[1] ;
		}
	}
}

void ATFMSectorDomain::absorbUAVTraffic(){
	for (list<UAV*>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		if ((*u)->loc==(*u)->end_loc){
			u = UAVs->erase(u) ;
		}
	}
}


void ATFMSectorDomain::getNewUAVTraffic(){
	// Create RAGS object with current weights history
	
	// Generates (with some probability) plane traffic for each sector
//	list<UAV> all_new_UAVs;
	list<UAV*> all_new_UAVs;
	
	// Collate weights for each type of UAV
	matrix3d w ;
	matrix2d w_e ;
	for (int i = 0; i < UAV::NTYPES; i++){
		w_e.clear() ;
		for (unsigned j = 0; j < edges.size(); j++){
			matrix1d w_mv(2) ;
			w_mv[0] = weights[0][i][j] ;
			w_mv[1] = weights[1][i][j] ;
			w_e.push_back(w_mv) ;
		}
		w.push_back(w_e) ;
	}
	
	// Call Fixes to generate traffic
	for (unsigned i=0; i<fixes->size(); i++){
		list<UAV*> new_UAVs = fixes->at(i).generateTraffic(fixes, obstacle_map, agent_locs, edges, w);
		all_new_UAVs.splice(all_new_UAVs.end(),new_UAVs);

		// obstacle check
		if (new_UAVs.size() && membership_map->at(new_UAVs.front()->loc)<0){
			printf("issue!");
			exit(1);
		}
	}
	UAVs->splice(UAVs->end(),all_new_UAVs);
}

void ATFMSectorDomain::reset(){
//	static int calls=0;

	// Drop all UAVs
	UAVs->clear();
	
	// Reset conflict counts
	conflict_count = 0;

	// Reset weights
	weights_history.clear() ;
	weights = matrix3d(2) ;
	matrix2d w_mean(UAV::NTYPES) ;
	matrix2d w_var(UAV::NTYPES) ;
	for (unsigned i=0; i<w_mean.size(); i++){
		w_mean[i] = DotMultiply(connection_time,2.0);
		w_var[i] = DotMultiply(connection_time,0.1);
	}
	weights[0] = w_mean ;
	weights[1] = w_var ;
	weights_history.push_back(weights[0]) ;
	
	// clear conflict count map
	for (int i=0; i<conflict_count_map->dim1(); i++){
		for (int j=0; j<conflict_count_map->dim2(); j++){
			conflict_count_map->at(i,j) = 0; // set all 0
		}
	}
}

void ATFMSectorDomain::logStep(int step){
}

void ATFMSectorDomain::exportLog(std::string fid, double G){
	static int calls = 0;
	PrintOut::toFileMatrix2D(*conflict_count_map,fid+to_string(calls)+".csv");
	calls++;
}

void ATFMSectorDomain::detectConflicts(){

	if (abstraction_mode){
		count_overcap();
	} else {
		double conflict_thresh = 1.0;
		for (list<UAV*>::iterator u1=UAVs->begin(); u1!=UAVs->end(); u1++){
			for (list<UAV*>::iterator u2=std::next(u1); u2!=UAVs->end(); u2++){
				double d = easymath::distance((*u1)->loc,(*u2)->loc);

				if (u1!=u2){
					if (d<conflict_thresh){
						conflict_count++;

						int avgx = ((*u1)->loc.x+(*u2)->loc.x)/2;
						int avgy = ((*u1)->loc.y+(*u2)->loc.y)/2;
						conflict_count_map->at(avgx,avgy)++;
						if ((*u1)->type_ID==UAV::FAST || (*u2)->type_ID==UAV::FAST){
							conflict_count+=10; // big penalty for high priority ('fast' here)
						}
					}
				}

			}
		}
	}
}
