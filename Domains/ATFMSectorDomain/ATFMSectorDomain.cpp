#include "ATFMSectorDomain.h"

using namespace std;
using namespace easymath;

UAV::UAV(XY start_loc, XY end_loc,std::vector<std::vector<XY> > *pathTraces, UAVType t):
	loc(start_loc), end_loc(end_loc), ID(pathTraces->size()), pathTraces(pathTraces), type_ID(t)
{
	pathTraces->push_back(vector<XY>(1,loc)); // new path trace created for each UAV,starting at location

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

void UAV::pathPlan(AStar_easy* Astar_highlevel, grid_lookup &m2astar, barrier_grid*obstacle_map,
				   ID_grid* membership_map, vector<Sector>* sectors, bool abstraction_mode, int connection_time[15][15])
{
	int memstart = membership_map->at(loc.x,loc.y);
	int memend = membership_map->at(end_loc.x,end_loc.y);
	
	list<AStar_easy::vertex> high_path = Astar_highlevel->search(memstart,memend);

	if (abstraction_mode){
		if (!high_path_prev.size()){ // new, put on time
			if (memstart==memend){
				t = 0;
			} else {
				t = connection_time[*high_path.begin()][*std::next(high_path.begin())];
			}
			high_path_prev = high_path;
		} else if (t<=0 && memstart==memend){ // on final leg; get to goal
			loc = end_loc;
		} else if (t<=0){ // middle leg; plan rest of path
			high_path.pop_front();
			loc = sectors->at(int(high_path.front())).xy; // now in a new sector!
			if (high_path.size()>1 && *high_path.begin()!=*std::next(high_path.begin())){
				t = connection_time[*high_path.begin()][*std::next(high_path.begin())];
			} else {
				t = 0;
			} 
		} else {
			t--;
		}

	} else {

		if (high_path_prev != high_path){ // Check if any course change necessary
			high_path_prev = high_path;


			int memnext;
			if (high_path.size()==1){
				memnext = high_path.front();
			} else {
				memnext = *std::next(high_path.begin());
			}
			XY waypoint;
			if (memnext==memend){
				waypoint = end_loc;
			} else {
				waypoint = sectors->at(memnext).xy;
			}

			// TODO: VERIFY HERE THAT THE HIGH_PATH_BEGIN() IS THE NEXT MEMBER... NOT THE FIRST...
			// target the center of the sector, or the goal if it is reachable
			vector<XY> low_path = m2astar[memstart][memnext]->m.get_solution_path(loc,waypoint);

			if (low_path.empty()) low_path.push_back(loc); // stay in place...

			while (target_waypoints.size()) target_waypoints.pop(); // clear the queue;
			for (int i=0; i<low_path.size(); i++) target_waypoints.push(low_path[i]); // adds waypoints to visit
			target_waypoints.pop(); // remove CURRENT location from target	
		}
	}

	/*
	if(high_path_prev !=high_path){
	high_path_prev = high_path;
	AStar_easy* astar_low;
	if (astar_lowlevel.count(high_path)){
	astar_low = astar_lowlevel[high_path];
	} else{
	// MAKE A NEW LOW-LEVEL ASTAR
	astar_low = new AStar_easy(high_path,obstacle_map,membership_map);
	astar_lowlevel[high_path] = astar_low;
	}
	list<AStar_easy::vertex> low_path= astar_low->search(loc,end_loc);
	while (target_waypoints.size()) target_waypoints.pop();
	for (list<AStar_easy::vertex>::iterator it=low_path.begin(); it!=low_path.end(); it++){
	int x, y;
	astar_low->ind2sub(astar_low->YDIM,*it, x,y);
	target_waypoints.push(XY(x,y)); // adds to list to visit
	}
	target_waypoints.pop(); // removes CURRENT location from target
	}
	*/
}

int UAV::getDirection(){
	// Identifies whether traveling in one of four cardinal directions
	return cardinalDirection(loc-end_loc);
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


	/*
	return u.target_waypoints.size()					// UAV has planned a trajectory
	&& u.target_waypoints.front()==loc				// UAV wants to go there next
	&& easymath::distance(u.loc,loc)<dist_thresh	// UAV is close enough
	&& u.end_loc==loc;								// This is destination fix
	*/
}

std::list<UAV> Fix::generateTraffic(vector<Fix>* allFixes, barrier_grid* obstacle_map,std::vector<std::vector<XY> > *pathTraces){
	static int calls = 0;
	// Creates a new UAV in the world
	std::list<UAV> newTraffic;
	
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
		newTraffic.push_back(UAV(loc,end_loc,pathTraces,type_id_set));
	}



	/* VARIABLE TRAFFIC METHOD
	double coin = COIN_FLOOR0;
	if (coin<p_gen){
		XY end_loc;
		do {
			end_loc = allFixes->at(COIN_FLOOR0*allFixes->size()).loc;
		} while (end_loc==loc);
		UAV::UAVType type_id_set = UAV::UAVType(calls%int(UAV::UAVType::NTYPES)); // EVEN TYPE NUMBER
		newTraffic.push_back(UAV(loc,end_loc,pathTraces,type_id_set));
	}*/

	calls++;
	return newTraffic;
}

void Fix::absorbTraffic(list<UAV>* UAVs){
	// INEFFICIENT: MOVED OUT OF FIX FUNCTION

	/*
	list<UAV> cur_UAVs;
	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		if (atDestinationFix(*u)){
			// don't copy over
		} else {
			if (u->target_waypoints.size() && u->target_waypoints.front()==loc){ // deleted if size==0; drop invalid plans
				u->target_waypoints.pop();
			}
			cur_UAVs.push_back(*u);
		}
	}
	(*UAVs) = cur_UAVs; // copy over
	*/
}

ATFMSectorDomain::ATFMSectorDomain(bool deterministic):
	is_deterministic(deterministic), abstraction_mode(true) // hardcode for the abstraction...
{

	pathTraces = new vector<vector<XY> >(); // traces the path (reset each epoch)

	// Object creation
	sectors = new vector<Sector>();
	fixes = new vector<Fix>();
	UAVs = new list<UAV>();

	// inherritance elements: constant
	//n_control_elements=4; // 4 outputs for sectors (cost in cardinal directions) (no types)
	n_control_elements=4*UAV::NTYPES;
	n_state_elements=4; // 4 state elements for sectors ( number of planes traveling in cardinal directions)
	n_steps=100; // steps of simulation time
	n_types=UAV::NTYPES;

	// Read in files for sector management
	obstacle_map = new barrier_grid(256,256);
	membership_map = new ID_grid(256,256); //HARDCODED
	//load_variable(*obstacle_map,"agent_map/obstacle_map.csv",0.0); // last element specifies height threshold for obstacle
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
	for (int i=0; i<agent_coords.size(); i++){
		sectors->push_back(Sector(XY(agent_coords[i][0],agent_coords[i][1])));
		agent_locs[i] = sectors->back().xy;
	}

	// Adjust the connection map to be the edges
	// preprocess boolean connection map
	// map edges to (sector ind, direction ind)
	for (int i=0; i<connection_map.size(); i++){
		for (int j=0; j<connection_map[i].size(); j++){
			if (connection_map[i][j] && i!=j){
				XY xyi = agent_locs[i];
				XY xyj = agent_locs[j];
				XY dx_dy = xyj-xyi;
				int xydir = cardinalDirection(dx_dy);
				int memj = membership_map->at(xyj); // only care about cost INTO sector
				sector_dir_map[edges.size()] = make_pair(memj,xydir); // add at new index
				edges.push_back(AStar_easy::edge(i,j));
			}
		}
	}

	//weights = vector<double>(edges.size(),1.0); //initialization of weights
	weights = matrix2d(UAV::NTYPES);
	for (int i=0; i<weights.size(); i++){
		weights[i] = matrix1d(edges.size(),1.0);
	}
	
	// initialize fixes
	for (int i=0; i<fix_locs.size(); i++){
		fixes->push_back(Fix(XY(fix_locs[i][0],fix_locs[i][1]),i,is_deterministic));
	}

	n_agents = sectors->size(); // number of agents dictated by read in file
	fixed_types=vector<int>(n_agents,0);

	conflict_count = 0; // initialize with no conflicts

	// Initialize Astar object (must re-create this each time weight change
	Astar_highlevel = std::vector<AStar_easy*>(UAV::NTYPES);
	for (int i=0; i<UAV::NTYPES; i++){
		Astar_highlevel[i] = new AStar_easy(agent_locs,edges,weights[i]);
	}

	// Add a different A* for each connection
	for (int i=0; i<connection_map.size(); i++){
		for (int j=0; j<connection_map[i].size(); j++){
			if (connection_map[i][j]>0){
				m2astar[i][j] = new AStar_grid(obstacle_map,membership_map,i,j);
			}
		}
	}

	
	conflict_count_map = new ID_grid(obstacle_map->dim1(), obstacle_map->dim2());


	if (abstraction_mode){
		if (n_agents!=15){
			printf("WRONG CONNECTION SIZE!");
			system("pause");
		}
		for (int i=0; i<n_agents; i++){
			for (int j=0; j<n_agents; j++){
				XY diff = agent_locs[i]-agent_locs[j];
				connection_time[i][j] = abs(diff.x)+abs(diff.y);
				for (int k=0; k<UAV::NTYPES; k++){
					connection_capacity[i][j][k] = 10; // flat capacity across all types on the connection
				}
			}
		}
	}
}

ATFMSectorDomain::~ATFMSectorDomain(void)
{
	delete UAVs;
	delete fixes;
	delete sectors;
	delete obstacle_map;
	delete membership_map;

	// Delete all pointers: recreate later...
	for (grid_lookup::iterator it=m2astar.begin(); it!=m2astar.end(); it++){
		for (std::map<int,AStar_grid*>::iterator inner=it->second.begin(); inner!=it->second.end(); inner++){
			delete inner->second;
		}
	}

	delete conflict_count_map;
	clear_all(Astar_highlevel);
	delete pathTraces;
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
		for (int i=0; i<overcap.size(); i++){
			for (int j=0; j<overcap[i].size(); j++){
				G -= overcap[i][j]*overcap[i][j]; // worse with higher concentration of planes!
				// clear overcap once used!
				overcap[i][j] = 0;
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

	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		allStates[getSector(u->loc)][u->getDirection()]+=1.0; // Adds the UAV impact on the state
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

	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		int a = getSector(u->loc);
		int id = u->type_ID;
		int dir = u->getDirection();
		if (a<0){
			printf("UAV %i at location %f,%f is in an obstacle.!", u->ID,u->loc.x,u->loc.y);
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

//HACK: ONLY GET PATH PLANS OF UAVS just generated
void ATFMSectorDomain::getPathPlans(){
	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		u->pathPlan(Astar_highlevel[u->type_ID],m2astar,obstacle_map,membership_map,sectors, abstraction_mode, connection_time); // sets own next waypoint
	}
}

void ATFMSectorDomain::getPathPlans(std::list<UAV> &new_UAVs){
	for (list<UAV>::iterator u=new_UAVs.begin(); u!=new_UAVs.end(); u++){
		u->pathPlan(Astar_highlevel[u->type_ID],m2astar,obstacle_map,membership_map,sectors, abstraction_mode, connection_time); // sets own next waypoint
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
	incrementUAVPath();
	detectConflicts();
	//printf("Conflicts %i\n",conflict_count);
}

void ATFMSectorDomain::setCostMaps(vector<vector<double> > agent_actions){
	// [next sector][direction of travel] -- current
	// agent_actions  = agent, [type, dir<-- alternating]

	for (int i=0; i<weights[0].size(); i++){
		for (int j=0; j<UAV::NTYPES; j++){
			int s = sector_dir_map[i].first;
			int d = j*(UAV::NTYPES-1) + sector_dir_map[i].second;
			
			// JEN: check for valid matrix indices
			if (s < 0 || s >= agent_actions.size())
				cout << "s: " << s << endl ;
			if (d < 0 || d >= agent_actions[s].size())
				cout << "d: " << d << endl ;
			weights[j][i] = agent_actions[s][d];
			if (weights[j][i] < 0.0){
				cout << "Negative weight at (j,i): (" << j << "," << i << ") (s,d): (" << s << "," << d << ")\n" ;
				cout << "weights[j][i]: " << weights[j][i] << " agent_actions[s][d]: " << agent_actions[s][d] << endl ;
			}
			// END
		}
		// HACK
		// weights[i] = agent_actions[sector_dir_map[i].first][sector_dir_map[i].second]; // AGENT SETUP // old
		// weights[i] = 1.0; // NO AGENTS
	}

	for (int i=0; i<Astar_highlevel.size(); i++){
		delete Astar_highlevel[i];
		Astar_highlevel[i] = new AStar_easy(agent_locs,edges,weights[i]); // replace existing weights
	}
}

void ATFMSectorDomain::incrementUAVPath(){
	if (!abstraction_mode){
		for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
			u->moveTowardNextWaypoint(); // moves toward next waypoint (next in low-level plan)
		}
	} else {
		// in abstraction mode, move to next center of target
		for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
			u->loc = sectors->at(*u->high_path_prev.begin()).xy;
			u->high_path_prev.pop_front();
		}
	}
	// IMPORTANT! At this point in the code, agent states may have changed
}

void UAV::moveTowardNextWaypoint(){
	for (int i=0; i<speed; i++){
		if (!target_waypoints.size()) return; // return if no waypoints
		loc = target_waypoints.front();
		pathTraces->at(ID).push_back(loc);
		target_waypoints.pop();
	}
}

void ATFMSectorDomain::absorbUAVTraffic(){
	// moved here for efficiency
	list<UAV> cur_UAVs;
	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		if (u->loc==u->end_loc){
			// don't copy over
		} else {

			// invalid plans no longer created
			/*if (u->target_waypoints.size() && u->target_waypoints.front()==loc){ // deleted if size==0; drop invalid plans
				u->target_waypoints.pop();
			}*/
			cur_UAVs.push_back(*u);
		}
	}
	(*UAVs) = cur_UAVs; // copy over


	/*for (int i=0; i<fixes->size(); i++){
		fixes->at(i).absorbTraffic(UAVs);
	}*/
}


void ATFMSectorDomain::getNewUAVTraffic(){
	//static int calls = 0;
	//static vector<XY> UAV_targets; // making static targets

	// Generates (with some probability) plane traffic for each sector
	list<UAV> all_new_UAVs;
	for (int i=0; i<fixes->size(); i++){
		list<UAV> new_UAVs = fixes->at(i).generateTraffic(fixes, obstacle_map,pathTraces);
		all_new_UAVs.splice(all_new_UAVs.end(),new_UAVs);

		// obstacle check
		if (new_UAVs.size() && membership_map->at(new_UAVs.front().loc)<0){
			printf("issue!");
			exit(1);
		}
	}

	getPathPlans(all_new_UAVs);

	UAVs->splice(UAVs->end(),all_new_UAVs);
	//calls++;
}

void ATFMSectorDomain::reset(){
	static int calls=0;

	// Drop all UAVs
	//printf("UAVs = %i\n",UAVs->size());
	UAVs->clear();
	// reset weights
	conflict_count = 0;
	//
	//weights = vector<double>(edges.size(),1.0); //initialization of weights
	weights = matrix2d(UAV::NTYPES);
	for (int i=0; i<weights.size(); i++){
		weights[i] = matrix1d(edges.size(), 1.0);
	}

	// re-create high level a*

	for (int i=0; i<Astar_highlevel.size(); i++){
		delete Astar_highlevel[i];
		Astar_highlevel[i] = new AStar_easy(agent_locs,edges,weights[i]);
	}

	// re-create low level a*
	/*for (map<list<AStar_easy::vertex>, AStar_easy* >::iterator it=astar_lowlevel.begin(); it!=astar_lowlevel.end(); it++){
	delete it->second;
	}
	astar_lowlevel.clear();*/

	//PrintOut::toFile(*conflict_count_map,"conflict_map.csv");
	// clear conflict count map
	for (int i=0; i<conflict_count_map->dim1(); i++){
		for (int j=0; j<conflict_count_map->dim2(); j++){
			conflict_count_map->at(i,j) = 0; // set all 0
		}
	}

	// reset the path trace so it doesn't get too big
	// pathTraces->clear();
}

void ATFMSectorDomain::logStep(int step){
	// log at 0 and 50
	// no logging, for speed
	/*
	if (step==0){
		pathSnapShot(0);
	}
	if (step==50){
		pathSnapShot(50);
		pathTraceSnapshot();
		//exit(1);
	}*/
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
		for (list<UAV>::iterator u1=UAVs->begin(); u1!=UAVs->end(); u1++){
			for (list<UAV>::iterator u2=std::next(u1); u2!=UAVs->end(); u2++){
				double d = easymath::distance(u1->loc,u2->loc);

				if (u1!=u2){
					if (d<conflict_thresh){
						conflict_count++;

						int avgx = (u1->loc.x+u2->loc.x)/2;
						int avgy = (u1->loc.y+u2->loc.y)/2;
						conflict_count_map->at(avgx,avgy)++;
						if (u1->type_ID==UAV::FAST || u2->type_ID==UAV::FAST){
							conflict_count+=10; // big penalty for high priority ('fast' here)
						}
					}
				}

			}
		}
	}
}

// PATH SNAPSHOT OUTPUT
void ATFMSectorDomain::pathSnapShot(int snapnum){
	matrix2d pathsnaps = matrix2d(2*UAVs->size());
	int ind = 0; // index of path given
	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		// pop through queue
		std::queue<XY> wpt_save = u->target_waypoints;
		while (u->target_waypoints.size()){
			pathsnaps[ind].push_back(u->target_waypoints.front().x);
			pathsnaps[ind+1].push_back(u->target_waypoints.front().y);
			u->target_waypoints.pop();
		}
		ind+=2;
		u->target_waypoints = wpt_save;
	}
	PrintOut::toFile2D(pathsnaps,"path-" + to_string(snapnum)+".csv");
}

void ATFMSectorDomain::pathTraceSnapshot(){
	// Exports all path traces
	matrix2d pathsnaps =matrix2d(2*pathTraces->size());
	for (int i=0, ind=0; i<pathTraces->size(); i++, ind+=2){
		for (int j=0; j<pathTraces->at(i).size(); j++){
			pathsnaps[ind].push_back(pathTraces->at(i)[j].x);
			pathsnaps[ind+1].push_back(pathTraces->at(i)[j].y);
		}
	}

	PrintOut::toFile2D(pathsnaps,"trace.csv");
}
