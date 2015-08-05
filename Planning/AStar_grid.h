#pragma once

#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <ctime>
#include <iostream>
#include <limits>
#include "../Math/easymath.h"
#include <sstream>
#include <fstream>
#include "../Math/Matrix.h"

using namespace easymath;
using namespace Numeric_lib;
using namespace std;

class mt{ // "maze types"
public:
	// Distance traveled in the maze
	typedef double distance;

	static const int GRID_RANK=2;
	typedef boost::grid_graph<GRID_RANK> grid;
	typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
	typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;

	// A hash function for vertices.
	struct vertex_hash:std::unary_function<vertex_descriptor, std::size_t> {
		std::size_t operator()(vertex_descriptor const& u) const {
			std::size_t seed = 0;
			boost::hash_combine(seed, u[0]);
			boost::hash_combine(seed, u[1]);
			return seed;
		}
	};

	typedef boost::unordered_set<vertex_descriptor, vertex_hash> vertex_set;
	typedef vector<vertex_descriptor> vertex_vector;
	typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type
		filtered_grid;

};

// Euclidean heuristic for a grid
//
// This calculates the Euclidean distance between a vertex and a goal
// vertex.
class euclidean_heuristic:
	public boost::astar_heuristic<mt::filtered_grid, double>
{
public:
  //euclidean_heuristic(mt::vertex_descriptor goal):m_goal(goal) {};
	euclidean_heuristic(){};

  double operator()(mt::vertex_descriptor v) {
	  return sqrt(pow(double(m_goal[0]) - double(v[0]), 2) + pow(double(m_goal[1]) - double(v[1]), 2));
  }

  mt::vertex_descriptor m_goal;
};

// A searchable maze
//
// The maze is grid of locations which can either be empty or contain a
// barrier.  You can move to an adjacent location in the grid by going up,
// down, left and right.  Moving onto a barrier is not allowed.  The maze can
// be solved by finding a path from the lower-left-hand corner to the
// upper-right-hand corner.  If no open path exists between these two
// locations, the maze is unsolvable.
//
// The maze is implemented as a filtered grid graph where locations are
// vertices.  Barrier vertices are filtered out of the graph.
//
// A-star search is used to find a path through the maze. Each edge has a
// weight of one, so the total path length is equal to the number of edges
// traversed.

// original: did not include following declarations required to allow static friend function
class maze ;
static std::ostream& operator<<(std::ostream&, const maze&) ;
// end amendments

class maze {
public:

		// Exception thrown when the goal vertex is found
	struct found_goal {};

	// Visitor that terminates when we find the goal vertex
	struct astar_goal_visitor:public boost::default_astar_visitor {
	//	astar_goal_visitor(mt::vertex_descriptor goal):m_goal(goal) {};
		astar_goal_visitor(){};

		void examine_vertex(mt::vertex_descriptor u, const mt::filtered_grid&) {
			if (u == m_goal)
				throw found_goal();
		}

		mt::vertex_descriptor m_goal;
	};


	friend std::ostream& operator<<(std::ostream&, const maze&);
	friend maze random_maze(std::size_t, std::size_t);

	// CREATION FUNCTIONS
	maze():m_grid(create_grid(0, 0)),m_barrier_grid(create_barrier_grid()) {};
	maze(std::size_t x, std::size_t y):m_grid(create_grid(x, y)),
		m_barrier_grid(create_barrier_grid()) {};

	maze(Matrix<bool,2> *obstacle_map):
		m_grid(create_grid(obstacle_map->dim1(),obstacle_map->dim2())),
		m_barrier_grid(create_barrier_grid())
	{
		int v_index = 0;
		// NOTE: V_INDEX COUNT ASSUMES YOU'RE GOING THROUGH THE OBSTACLES DIFFERENTLY...
		//for (std::vector<std::vector<bool> >::iterator obs = obstacle_map->begin(); obs!=obstacle_map->end(); obs++){
			//for (std::vector<bool>::iterator o = obs->begin(); o!=obs->end(); o++){
		for (int y=0; y<obstacle_map->dim2(); y++){
			for (int x=0; x<obstacle_map->dim1(); x++){
				if ((*obstacle_map)(x,y)){ // there is an obstacle!
					mt::vertex_descriptor u = vertex(v_index,m_grid);
					m_barriers.insert(u); // insert a barrier!
				}
				v_index++; // increment v_index even if no barrier added!
			}
		}
	}

	maze(std::vector<std::vector<bool> >* obstacle_map):
		m_grid(create_grid(obstacle_map->size(),obstacle_map->begin()->size())),
		m_barrier_grid(create_barrier_grid())
	{
		// SOURCE AND GOAL DECIDED IN SEARCH!

		int v_index = 0;
		// NOTE: V_INDEX COUNT ASSUMES YOU'RE GOING THROUGH THE OBSTACLES DIFFERENTLY...
		//for (std::vector<std::vector<bool> >::iterator obs = obstacle_map->begin(); obs!=obstacle_map->end(); obs++){
			//for (std::vector<bool>::iterator o = obs->begin(); o!=obs->end(); o++){
		for (int y=0; y<obstacle_map->begin()->size(); y++){
			for (int x=0; x<obstacle_map->size(); x++){
				if (obstacle_map->at(x)[y]){ // there is an obstacle!
					mt::vertex_descriptor u = vertex(v_index,m_grid);
					m_barriers.insert(u); // insert a barrier!
				}
				v_index++; // increment v_index even if no barrier added!
			}
		}
	}

	maze(std::vector<std::vector<bool> >* obstacle_map, std::vector<std::vector<int> >* membership_map, int m1, int m2):
		m_grid(create_grid(obstacle_map->size(),obstacle_map->begin()->size())),
		m_barrier_grid(create_barrier_grid())
	{
		// This map will only show areas of membership. Others are seen as barriers.

		int v_index = 0;
		for (int x=0; x<obstacle_map->begin()->size(); x++){
			for (int y=0; y<obstacle_map->size(); y++){
				if (obstacle_map->at(x)[y] || (membership_map->at(x)[y]!=m1 && membership_map->at(x)[y]!=m2)){ // there is an obstacle, or wrong membership
					mt::vertex_descriptor u = vertex(v_index,m_grid);
					m_barriers.insert(u); // insert a barrier!
				}
				v_index++; // increment v_index even if no barrier added!
			}
		}
	}

	maze(Matrix<bool,2> *obstacle_map, Matrix<int,2> *members, int m1, int m2):
		m_grid(create_grid(obstacle_map->dim1(), obstacle_map->dim2())),
		m_barrier_grid(create_barrier_grid())
	{
		// This map will only show areas of membership. Others are seen as barriers.
		int v_index = 0;
		for (int y=0; y<obstacle_map->dim2(); y++){
			for (int x=0; x<obstacle_map->dim1(); x++){
				if (((*obstacle_map)(x,y)) || 
					((*members)(x,y)!=m1 && (*members)(x,y)!=m2)){ // there is an obstacle, or wrong membership
					mt::vertex_descriptor u = vertex(v_index,m_grid);
					m_barriers.insert(u); // insert a barrier!
				}
				v_index++; // increment v_index even if no barrier added!
			}
		}
	}

	// The length of the maze along the specified dimension.
	mt::vertices_size_type length(std::size_t d) const {return m_grid.length(d);}

	bool has_barrier(mt::vertex_descriptor u) const {
		return m_barriers.find(u) != m_barriers.end();
	}

	double solve(int xsource, int ysource, int xgoal, int ygoal){
		boost::static_property_map<mt::distance> weight(1);
		// The predecessor map is a vertex-to-vertex mapping.
		typedef boost::unordered_map<mt::vertex_descriptor,
			mt::vertex_descriptor,
			mt::vertex_hash> pred_map;
		pred_map predecessor;
		boost::associative_property_map<pred_map> pred_pmap(predecessor);
		// The distance map is a vertex-to-distance mapping.
		typedef boost::unordered_map<mt::vertex_descriptor,
			mt::distance, mt::vertex_hash> dist_map;
		dist_map distance;
		boost::associative_property_map<dist_map> dist_pmap(distance);

		mt::vertex_descriptor s = {{xsource, ysource}};
		mt::vertex_descriptor g = {{xgoal, ygoal}};
		heuristic.m_goal = g;
		visitor.m_goal = g;
		m_solution.clear();

		try {
			astar_search(m_barrier_grid, s, heuristic,
				boost::weight_map(weight).
				predecessor_map(pred_pmap).
				distance_map(dist_pmap).
				visitor(visitor) );
		} catch(found_goal fg) {
			// Walk backwards from the goal through the predecessor chain adding
			// vertices to the solution path.
			for (mt::vertex_descriptor u = g; u != s; u = predecessor[u])
				m_solution.push_back(u);
			m_solution.push_back(s);
			m_solution_length = distance[g];
			//return true;
			return m_solution_length;
		}
		double maxdist = DBL_MAX;
		return maxdist;
	}

	std::vector<XY> get_solution_path(XY source, XY goal){
		solve(source.x,source.y, goal.x, goal.y);

//		printf("size = %i\n",m_solution.size());

		std::vector<XY> soln;
		for (mt::vertex_vector::iterator it=m_solution.begin(); it!=m_solution.end(); it++){
			soln.push_back(XY(it->front(), it->back()));
		}
		return soln;
	}

	bool solved() const {return !m_solution.empty();}
	bool solution_contains(mt::vertex_descriptor u) const {
		return std::find(m_solution.begin(),m_solution.end(),u) != m_solution.end();
	}

	euclidean_heuristic heuristic;
	astar_goal_visitor visitor;

	
	// The length of the solution path
	mt::distance m_solution_length;

	void printMap(std::string dir, int label1, int label2){
		stringstream ss;
		ss << dir << label1 << "-" <<label2 << ".csv";
		ofstream output(ss.str());

		
		for (int i = 0; i<length(0); i++){
			for (int j = 0; j < length(1); j++) {
				// Put the character representing this point in the maze grid.
				mt::vertex_descriptor u = {{i, j}};
				if (solution_contains(u))
					output << 2 << ",";
				else if (has_barrier(u))
					output << 1 << ",";
				else
					output << 0 << ",";
			}
			output << std::endl;
		}
	}
		// The vertices on a solution path through the maze
	mt::vertex_vector m_solution;

private:
	// Create the underlying rank-2 grid with the specified dimensions.
	mt::grid create_grid(std::size_t x, std::size_t y) {
		boost::array<std::size_t, mt::GRID_RANK> lengths = { {x, y} };
		return mt::grid(lengths);
	}

	// Filter the barrier vertices out of the underlying grid.
	mt::filtered_grid create_barrier_grid() {
		return boost::make_vertex_subset_complement_filter(m_grid, m_barriers);
	}

	// The grid underlying the maze
	mt::grid m_grid;
	// The underlying maze grid with barrier vertices filtered out
	mt::filtered_grid m_barrier_grid;
	// The barriers in the maze
	mt::vertex_set m_barriers;

	
};

static std::ostream& operator<<(std::ostream& output, const maze& m) {
		// Header
		std::string BARRIER = "#";

		//for (mt::vertices_size_type i = 0; i < m.length(0)+2; i++)

		int START = 78;
		int END = START+78;

		for (mt::vertices_size_type i = START; i < END+2; i++)
			output << BARRIER;
		output << std::endl;
		// Body
		for (int y = 0; y<m.length(1); y++){//m.length(1)-1; y >= 0; y--) {
			// Enumerate rows in reverse order and columns in regular order so that
			// (0,0) appears in the lower left-hand corner.  This requires that y be
			// int and not the unsigned vertices_size_type because the loop exit
			// condition is y==-1.
			//for (mt::vertices_size_type x = 0; x < m.length(0); x++) { // hack to fit on screen
			for (mt::vertices_size_type x = START; x < END; x++) {
				// Put a barrier on the left-hand side.
				if (x == 0)
					output << BARRIER;
				// Put the character representing this point in the maze grid.
				mt::vertex_descriptor u = {{x, mt::vertices_size_type(y)}};
				if (m.solution_contains(u))
					output << ".";
				else if (m.has_barrier(u))
					output << BARRIER;
				else
					output << " ";
				// Put a barrier on the right-hand side.
				if (x == m.length(0)-1)
					output << BARRIER;
			}
			// Put a newline after every row except the last one.
			output << std::endl;
		}
		// Footer
		for (mt::vertices_size_type i = START; i < END+2; i++)
			output << BARRIER;
		if (m.solved())
			output << std::endl << "Solution length " << m.m_solution_length;
		return output;
}


class AStar_grid
{
public:
	AStar_grid(void);
	AStar_grid(std::vector<std::vector<bool> > *obstacle_map):
		m(maze(obstacle_map))
	{
	}
	AStar_grid(std::vector<std::vector<bool> > *obstacle_map, std::vector<std::vector<int> >* members, int m1, int m2):
		m(maze(obstacle_map,members,m1,m2))
	{
	}
	AStar_grid(Matrix<bool,2> *obstacle_map, Matrix<int,2> *members, int m1, int m2):
		m(maze(obstacle_map,members,m1,m2))
	{
	}
	AStar_grid(Matrix<bool,2> *obstacle_map):
		m(maze(obstacle_map))
	{
	}
	~AStar_grid(void){};
	maze m;
};

/*



// Solve the maze using A-star search.  Return true if a solution was found.
bool maze::solve() {
}


*/

