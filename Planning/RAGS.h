#pragma once

#include <iostream>
#include <random>
#include <utility>
#include <algorithm>
#include <time.h>
#include <float.h>
#include <limits.h>
#include <fstream>
#include <sstream>

#include "../Math/easymath.h"

using namespace easymath ;
using namespace std ;

typedef unsigned long int ULONG ;
enum searchType {ASTAR, DIJKSTRA} ; // BREADTH, DEPTH
enum heuristic {ZERO, MANHATTAN, EUCLIDEAN} ;
enum pathOut {BEST,ALL} ;
enum nodeType {SOURCE, OTHER} ;

const double pi = 3.14159265358979323846264338328 ;

// Search configuration
//searchType SEARCH_TYPE = ASTAR ;
//heuristic HEURISTIC = ZERO ;
//pathOut PSET = ALL ;

// Vertex class to contain location of vertices
class Node ;
class Vertex
{
	public:
		Vertex(double x, double y):
		  itsX(x), itsY(y) {}
		~Vertex() {}
		
		double GetX() const {return itsX ;}
		void SetX(double x) {itsX = x ;}
		double GetY() const {return itsY ;}
		void SetY(double y) {itsY = y ;}
		double GetCTC()const{return itsCTC ;}
		void SetCTC(double cost_to_come) {itsCTC = cost_to_come ;}
		void SetNodes(vector<Node *> NewNodes) {itsNodes = NewNodes ;}
		vector<Node *> GetNodes() {return itsNodes ;}
		void SetActualCost(double ac) {itsActualCost = ac;}
		double GetActualCost() const {return itsActualCost ;}
		
		friend bool operator==(const Vertex &lhs, const Vertex &rhs){
			return lhs.GetX()==rhs.GetX() && lhs.GetY()==rhs.GetY();
		}

	private:
		double itsActualCost ;
		double itsX ;
		double itsY ;
		double itsCTC ;
		vector<Node *> itsNodes ;
};

// Edge class to contain mean and variance of cost along an edge
class Edge
{
	public:
		Edge(Vertex * v1, Vertex * v2, double cost, double var):
		  itsVertex1(v1), itsVertex2(v2), itsMeanCost(cost), itsVarCost(var), itsMeanSearch(cost), itsVarSearch(var) {}
		~Edge()
		{
	    delete itsVertex1 ;
	    itsVertex1 = 0 ;
	    delete itsVertex2 ;
	    itsVertex2 = 0 ;
    }
		
		Vertex * GetVertex1() const {return itsVertex1 ;}
		Vertex * GetVertex2() const {return itsVertex2 ;}
		double GetMeanCost() const {return itsMeanCost ;}
		void SetMeanCost(double cost) {itsMeanCost = cost ;}
		double GetVarCost() const {return itsVarCost ;}
		void SetVarCost(double var) {itsVarCost = var ;}
		double GetTrueCost() {return itsTrueCost ;}
		void SetTrueCost(double cost) {itsTrueCost = cost ;}
		double GetMeanSearch() const {return itsMeanSearch ;}
		void SetMeanSearch(double cost) {itsMeanSearch = cost ;}
		double GetVarSearch() const {return itsVarSearch ;}
		void SetVarSearch(double var) {itsVarSearch = var ;}
		void SetTrueCost(default_random_engine generator) ;
	private:
		Vertex * itsVertex1 ;
		Vertex * itsVertex2 ;
		double itsMeanCost ;
		double itsVarCost ;
		double itsTrueCost ;
		double itsMeanSearch ; // Actual value used in search
		double itsVarSearch ; // Actual value used in search
} ;

// Graph class to create and store graph structure
class Graph
{
	public:
	  typedef pair<int, int> edge ;
	  Graph(vector<XY> &locations, vector<edge> &edge_array, vector< vector<double> > &weights)
	  {
	    itsVertices = GenerateVertices(locations) ;
	    itsEdges = GenerateEdges(edge_array, weights) ;
    }
    
		~Graph()
		{
	    delete [] itsVertices ;
	    itsVertices = 0;
	    delete [] itsEdges ;
	    itsEdges = 0 ;
    }
		
		Vertex ** GetVertices() const {return itsVertices ;}
		Edge ** GetEdges() const {return itsEdges ;}
		ULONG GetNumVertices() const {return numVertices ;}
		ULONG GetNumEdges() const {return numEdges ;}
		
		vector<Edge *> GetNeighbours(XY v) ;
		vector<Edge *> GetNeighbours(Vertex * v) ;
		vector<Edge *> GetNeighbours(XY v, XY v0) ;
		vector<Edge *> GetNeighbours(Vertex * v, Vertex * v0) ;
		vector<Edge *> GetNeighbours(Node * n) ;
		
	private:
		Vertex ** itsVertices ;
		Edge ** itsEdges ;
		ULONG numVertices ;
		ULONG numEdges ;
		
		Vertex ** GenerateVertices(vector<XY> &vertices) ;
		Edge ** GenerateEdges(vector<edge> &edges, vector< vector<double> > &weights) ;
} ;

// Node class to maintain path information up to a vertex
// contains parent node, (mu, sigma) of cost-to-come
class Node
{
	public:
		Node(Vertex * vertex):
		  itsVertex(vertex), itsParent(0), itsMeanCost(0.0), itsVarCost(0.0), itsDepth(0),
		  itsHeuristic(0.0), itsMeanCTG(0.0), itsVarCTG(0.0) {}
		
		Node(Vertex * vertex, nodeType n):
		itsVertex(vertex), itsParent(0), itsHeuristic(0.0)
		{
	    switch (n)
	    {
		    case SOURCE:
			    itsMeanCost = 0.0 ;
			    itsVarCost = 0.0 ;
			    itsDepth = 0.0 ;
			    itsMeanCTG = 0.0 ;
			    break ;
		    default:
			    itsMeanCost = DBL_MAX ;
			    itsVarCost = DBL_MAX ;
			    itsDepth = ULONG_MAX ;
			    itsMeanCTG = DBL_MAX ;
			    itsVarCTG = DBL_MAX ;
	    }
    }
    
		Node(Node * parent, Edge * edge):
		itsParent(parent), itsHeuristic(0.0), itsMeanCTG(0.0), itsVarCTG(0.0)
		{
	    itsVertex = edge->GetVertex2() ;
	    itsMeanCost = itsParent->GetMeanCost() + edge->GetMeanSearch() ;
	    itsVarCost = itsParent->GetVarCost() + edge->GetVarSearch() ;
	    itsDepth = itsParent->GetDepth() + 1 ;
    }
    
		~Node(){} ;
		
		Node * GetParent() const {return itsParent ;}
		void SetParent(Node * parent) {itsParent = parent ;}
		double GetMeanCost() const {return itsMeanCost ;}
		void SetMeanCost(double cost) {itsMeanCost = cost ;}
		double GetVarCost() const {return itsVarCost ;}
		void SetVarCost(double var) {itsVarCost = var ;}
		Vertex * GetVertex() const {return itsVertex ;}
		void SetVertex(Vertex * vertex) {itsVertex = vertex ;}
		ULONG GetDepth() const {return itsDepth ;}
		void SetDepth(ULONG depth) {itsDepth = depth ;}
		double GetHeuristic() const {return itsHeuristic ;}
		void SetHeuristic(double h) {itsHeuristic = h ;}
		double GetMeanCTG() const {return itsMeanCTG ;}
		double GetVarCTG() const {return itsVarCTG ;}
		
		void DisplayPath() ;
		Node * ReverseList(Node * itsChild) ;
		void SetCTG(double totalMean, double totalVar) ;
    
	private:
		Vertex * itsVertex ;
		Node * itsParent ;
		double itsMeanCost ;
		double itsVarCost ;
		ULONG itsDepth ;
		double itsHeuristic ;
		double itsMeanCTG ;
		double itsVarCTG ;
} ;

// Node comparison class
class CompareNode
{
	public:
		bool operator() (const Node * n1, const Node * n2) const
		{
//			switch (SEARCH_TYPE)
//				{
//					case BREADTH:
//						return (n1->GetDepth() > n2->GetDepth()) ;
//					default:
						double n1Cost = n1->GetMeanCost() + n1->GetHeuristic() ;
						double n2Cost = n2->GetMeanCost() + n2->GetHeuristic() ;
						return (n1Cost >= n2Cost && n1->GetVarCost() >= n2->GetVarCost()) ;
//						if (n1Cost > n2Cost && n1->GetVarCost() > n2->GetVarCost())
//							return true ;
//						else if (n2Cost > n1Cost && n2->GetVarCost() > n1->GetVarCost())
//							return false ;
//						else
//							return (n1Cost > n2Cost) ;
//				}
		}
} ;

// Custom queue type to perform priority queue updates
class Queue
{
	public:
  	typedef priority_queue<Node *, vector<Node *>, CompareNode> QUEUE ;
		Queue(Node * source){
	    itsPQ = new QUEUE ;
	    itsPQ->push(source) ;
    }
    
		~Queue(){
			while (!itsPQ->empty())
				itsPQ->pop() ;
	    delete itsPQ ;
	    itsPQ = 0 ;
	    for (ULONG i = 0; i < closed.size(); i ++){
		    delete closed[i] ;
		    closed[i] = 0 ;
	    }
    }
		
		vector<Node *> GetClosed() const {return closed ;}
		bool EmptyQueue() const {return itsPQ->empty() ;}
		ULONG SizeQueue() const {return (ULONG)itsPQ->size() ;}
		void UpdateQueue(Node * newNode) ;
		Node * PopQueue() ;
    
	private:
		QUEUE * itsPQ ;
		vector<Node *> closed ;
		
		bool CompareNodes(const Node * n1, const Node * n2) const ;
} ;

// Path search class to store and search a graph
// A* search with zero, Manhattan or Euclidean distance heuristics
class Search
{
	public:
		Search(Graph * graph, Vertex * source, Vertex * goal):itsGraph(graph), itsSource(source), itsGoal(goal){
	  	SEARCH_TYPE = ASTAR ;
	  	HEURISTIC = ZERO ;
  	}

		~Search(){
			printf("Called Search destructor\n") ;
			// These may cause segfault later on, need to run for longer
			delete itsGraph ;
			itsGraph = 0 ;
			delete itsSource ;
			itsSource = 0 ;
			delete itsGoal ;
			itsGoal = 0 ;
			// END
	    delete itsQueue ;
	    itsQueue = 0 ;
    }
		
		Graph * GetGraph() const {return itsGraph ;}
		Queue * GetQueue() const {return itsQueue ;}
		void SetQueue(Queue * queue) {itsQueue = queue ;}
		Vertex * GetSource() const {return itsSource ;}
		Vertex * GetGoal() const {return itsGoal ;}
		vector<Node *> PathSearch(pathOut pType) ;

	private:
		Graph * itsGraph ;
		Queue * itsQueue ;
		Vertex * itsSource ;
		Vertex * itsGoal ;
		searchType SEARCH_TYPE ;
		heuristic HEURISTIC ;
		
		ULONG FindSourceID() ;
		double ManhattanDistance(Vertex * v1, Vertex * v2) ;
		double EuclideanDistance(Vertex * v1, Vertex *v2) ;
		void UpdateNode(Node * n) ;
} ;

// RAGS class for interfacing with sector agents
class RAGS
{
  public:
    typedef pair<int, int> edge ;
    RAGS(vector<XY> &locations, vector<edge> &edge_array, vector< vector<double> > &weights): 
    itsLocations(locations), itsEdgeArray(edge_array){
    	itsGraph = new Graph(locations, edge_array, weights) ;
    	PSET = ALL ;
    }
    
    ~RAGS()
    {
      delete itsGraph ;
      itsGraph = 0 ;
      delete itsSearch ;
      itsSearch = 0 ;
    }
    
    Graph * GetGraph() const {return itsGraph ;}
    Vertex * GetVert() const {return itsVert ;}
    
    void SetInitialVert(Vertex * start) ;
    XY SearchGraph(XY start, XY goal, vector<double> &weights) ;
    XY SearchGraph(Vertex * start, Vertex * goal, vector<double> &weights) ;
    int GetEdgeIndex(XY start, XY goal) ;
    
  private:
  	vector<XY> itsLocations ;
    vector<edge> itsEdgeArray ;
    Graph * itsGraph ;
    Search * itsSearch ;
    Vertex * itsVert ;
    vector<Node *> itsNDSet ;
    pathOut PSET ;
    
    void AssignCurrentEdgeCosts(vector<double> &weights) ;
} ;
