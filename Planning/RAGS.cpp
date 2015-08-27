#include "RAGS.h"

void Edge::SetTrueCost(default_random_engine generator)
{
  double diff_x = itsVertex1->GetX() - itsVertex2->GetX() ;
  double diff_y = itsVertex1->GetY() - itsVertex2->GetY() ;
  double diff = sqrt(pow(diff_x,2) + pow(diff_y,2)) ;
  normal_distribution<double> distribution(itsMeanCost,itsVarCost) ;
  itsTrueCost = distribution(generator) ;
  if (itsTrueCost < diff)
    itsTrueCost = diff ;
}

vector<Edge *> Graph::GetNeighbours(XY v)
{
  Vertex * pV ;
  for (ULONG i = 0; i < numVertices; i++)
  {
    if ( v.x == itsVertices[i]->GetX() && v.y == itsVertices[i]->GetY() )
      pV = itsVertices[i] ;
  }
  return GetNeighbours(pV) ;
}

vector<Edge *> Graph::GetNeighbours(Vertex * v)
{
  vector<Edge *> neighbours(numEdges) ;
  ULONG k = 0 ;

  for (ULONG i = 0; i < numEdges; i++)
  {
	  double x1 = itsEdges[i]->GetVertex1()->GetX() ;
	  double y1 = itsEdges[i]->GetVertex1()->GetY() ;
	  if (x1 == v->GetX() && y1 == v->GetY())
	  {
		  neighbours[k] = itsEdges[i] ;
		  k++ ;
	  }
  }

  neighbours.resize(k) ;
  return neighbours ;
}

Vertex ** Graph::GenerateVertices(vector<XY> &vertices)
{
  numVertices = (ULONG)vertices.size() ;
  Vertex ** allVertices = new Vertex * [numVertices] ;
	
  for (ULONG i = 0; i < numVertices; i++)
  {
    double x = vertices[i].x ;
    double y = vertices[i].y ;

    allVertices[i] = new Vertex(x,y) ;
  }
  
  return allVertices ;
}

Edge ** Graph::GenerateEdges(vector<edge> &edges, vector< vector<double> > &weights)
{
  numEdges = (ULONG)edges.size() ;
  Edge ** allEdges = new Edge * [numEdges] ;

  for (ULONG i = 0; i < numEdges; i++)
  {
    Vertex * v1 = itsVertices[(ULONG)edges[i].first] ;
    Vertex * v2 = itsVertices[(ULONG)edges[i].second] ;
    double cost = weights[i][0] ;
    double var = weights[i][1] ;

    allEdges[i] = new Edge(v1, v2, cost, var) ;
  }

  return allEdges ;
}

void Node::DisplayPath()
{
  cout << "Vertex: (" << itsVertex->GetX() << "," << itsVertex->GetY() << ")\n" ;
  cout << "Mean cost-to-come: " << itsMeanCost << ", " << "variance: " << itsVarCost << endl ;
  cout << "Mean cost-to-go: " << itsMeanCTG << ", " << "variance: " << itsVarCTG << endl ;

  if (itsParent)
    itsParent->DisplayPath() ;
}

Node * Node::ReverseList(Node * itsChild)
{
  Node * itsParentR = new Node(GetVertex()) ;
  itsParentR->SetMeanCost(GetMeanCost()) ;
  itsParentR->SetVarCost(GetVarCost()) ;
  itsParentR->SetParent(itsChild) ;

  Node * itsReverse ;

  if (GetParent())
  {
    itsReverse = GetParent()->ReverseList(itsParentR) ;
    return itsReverse ;
  }
  else
    return itsParentR ;
}

void Node::SetCTG(double totalMean, double totalVar)
{
  itsMeanCTG = totalMean - itsMeanCost ;
  itsVarCTG = totalVar - itsVarCost ;

  if (itsParent)
    itsParent->SetCTG(totalMean, totalVar) ;
}

void Queue::UpdateQueue(Node * newNode)
{
  // Compare newNode to nodes in closed set
  // if closed contains node with same vertex, compare their costs
  // choose whether or not to create a new node
  bool dom = false ;
  for (ULONG i = 0; i < closed.size(); i++)
  {
    if (closed[i]->GetVertex() == newNode->GetVertex())
    {
	    dom = CompareNodes(newNode, closed[i]) ;
	    if (dom)
		    return ;
    }
  }
  itsPQ->push(newNode) ;
}

Node * Queue::PopQueue()
{
  // Check if next node is already dominated by existing node in closed set
  Node * newNode = itsPQ->top() ;
  bool dom = false ;
  for (ULONG i = 0; i < closed.size(); i++)
  {
    if (closed[i]->GetVertex() == newNode->GetVertex())
    {
      dom = CompareNodes(newNode, closed[i]) ;
      if (dom)
      {
	      itsPQ->pop() ;
	      return 0 ;
      }
    }
  }
  closed.push_back(itsPQ->top()) ;
  itsPQ->pop() ;
  return closed[(ULONG)closed.size()-1] ;
}

bool Queue::CompareNodes(const Node * n1, const Node * n2) const
{
  double n1Cost = n1->GetMeanCost() ;
  double n2Cost = n2->GetMeanCost() ;
  return (n1Cost >= n2Cost && n1->GetVarCost() >= n2->GetVarCost()) ;
  /*	if (n1Cost > n2Cost && n1->GetVarCost() > n2->GetVarCost())
    return true ;
  else if (n2Cost > n1Cost && n2->GetVarCost() > n1->GetVarCost())
    return false ;
  else
    return (n1Cost > n2Cost) ;*/
}

vector<Node *> Search::PathSearch(pathOut pType)
{
  ULONG sourceID = FindSourceID() ;
  itsQueue = new Queue(new Node(itsGraph->GetVertices()[sourceID], SOURCE)) ;

  clock_t t_start = clock() ;
  double t_elapse = 0.0 ;

  while (!itsQueue->EmptyQueue() && t_elapse < 5)
  {
    // Pop cheapest node from queue
    Node * currentNode = itsQueue->PopQueue() ;
    if (!currentNode) { continue ; }

    if (pType == BEST)
    {
	    // Terminate search once one path is found
	    if (currentNode->GetVertex() == itsGoal)
		    break ;
    }

    Node * currentNeighbour ;

    // Find all neighbours
    vector<Edge *> neighbours = itsGraph->GetNeighbours(currentNode->GetVertex()) ;

    // Update neighbours
    for (ULONG i = 0; i < (ULONG)neighbours.size(); i++)
    {
	    // Check if neighbour vertex is already in closed set
	    bool newNeighbour = true ;
	    if (pType == BEST)
	    {
		    Vertex * vcheck = neighbours[i]->GetVertex2() ;
		    for (ULONG j = 0; j < itsQueue->GetClosed().size(); j++)
		    {
			    if (itsQueue->GetClosed()[j]->GetVertex() == vcheck)
			    {
				    newNeighbour = false ;
				    break ;
			    }
		    }
	    }
	
	    if (newNeighbour)
	    {
		    // Create neighbour node
		    currentNeighbour = new Node(currentNode, neighbours[i]) ;
		    UpdateNode(currentNeighbour) ;
		    itsQueue->UpdateQueue(currentNeighbour) ;
	    }
    }

    t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;
  }

  // Check if a path is found
  bool ClosedAll = false ;
  for (ULONG i = 0; i < itsQueue->GetClosed().size(); i++)
  {
    if (itsQueue->GetClosed()[i]->GetVertex() == itsGoal)
    {
	    ClosedAll = true ;
	    break ;
    }
  }

  if (!ClosedAll)
  {
    cout << "No path found from source to goal.\n" ;
    vector<Node *> bestPath ;
    return bestPath ;
  }
  else
  {
    ULONG k = 0 ;
    vector<Node *> bestPath((ULONG)itsQueue->GetClosed().size()) ;

    for (ULONG i = 0; i < (ULONG)itsQueue->GetClosed().size(); i++)
    {
	    if (itsGoal == itsQueue->GetClosed()[i]->GetVertex())
	    {
		    bestPath[k] = itsQueue->GetClosed()[i] ;
		    k++ ;
	    }
    }

    bestPath.resize(k) ;
    return bestPath ;
  }
}

ULONG Search::FindSourceID()
{
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
    if (itsSource == itsGraph->GetVertices()[i])
      return i ;
  cout << "Error: souce ID not found. Exiting.\n" ;
  exit(1) ;
}

double Search::ManhattanDistance(Vertex * v1, Vertex * v2)
{
  double diffX = abs(v1->GetX() - v2->GetX()) ;
  double diffY = abs(v1->GetY() - v2->GetY()) ;
  double diff = diffX + diffY ;
  return diff ;
}

double Search::EuclideanDistance(Vertex * v1, Vertex *v2)
{
  double diffX = pow(v1->GetX() - v2->GetX(),2) ;
  double diffY = pow(v1->GetY() - v2->GetY(),2) ;
  double diff = sqrt(diffX+diffY) ;
  return diff ;
}

void Search::UpdateNode(Node * n)
{
  if (SEARCH_TYPE == ASTAR)
  {
    double diff ;
    switch (HEURISTIC)
    {
	    case ZERO:
		    diff = 0.0 ;
		    n->SetHeuristic(diff) ;
		    break ;
	    case MANHATTAN:
		    diff = ManhattanDistance(itsGoal, n->GetVertex()) ;
		    n->SetHeuristic(diff) ;
		    break ;
	    case EUCLIDEAN:
		    diff = EuclideanDistance(itsGoal, n->GetVertex()) ;
		    n->SetHeuristic(diff) ;
		    break ;
    }
  }
  else if (SEARCH_TYPE == DIJKSTRA)
  {
    HEURISTIC = ZERO ;
    SEARCH_TYPE = ASTAR ;
    UpdateNode(n) ;
  }
}

vector<double> linspace(double a, double b, int n)
{
  vector<double> array ;
  double step = (b-a)/(n-1) ;
  while (a<=b)
  {
    array.push_back(a) ;
    a += step ;
  }
  return array ;
}

// Function to compare vertices: returns TRUE if vertex A is better than vertex B
bool ComputeImprovementProbability(Vertex * A, Vertex * B)
{
  vector<Node *> ANodes = A->GetNodes();
  vector<Node *> BNodes = B->GetNodes();

  double c_A0 = A->GetCTC() ;
  double c_B0 = B->GetCTC() ;
  double max_3sig = ANodes[0]->GetMeanCTG() + 3*ANodes[0]->GetVarCTG() ;
  double min_3sig = ANodes[0]->GetMeanCTG() - 3*ANodes[0]->GetVarCTG() ;
  for (unsigned i = 0; i < ANodes.size(); i++)
  {
    if (max_3sig < ANodes[i]->GetMeanCTG()+3*ANodes[i]->GetVarCTG())
      max_3sig = ANodes[i]->GetMeanCTG()+3*ANodes[i]->GetVarCTG() ;
    if (min_3sig > ANodes[i]->GetMeanCTG()-3*ANodes[i]->GetVarCTG())
      min_3sig = ANodes[i]->GetMeanCTG()-3*ANodes[i]->GetVarCTG() ;
  }
  for (unsigned i = 0; i < BNodes.size(); i++)
  {
    if (max_3sig < BNodes[i]->GetMeanCTG()+3*BNodes[i]->GetVarCTG())
      max_3sig = BNodes[i]->GetMeanCTG()+3*BNodes[i]->GetVarCTG() ;
    if (min_3sig > BNodes[i]->GetMeanCTG()-3*BNodes[i]->GetVarCTG())
      min_3sig = BNodes[i]->GetMeanCTG()-3*BNodes[i]->GetVarCTG() ;
  }

  // If code is taking too long, change n to a smaller value
  // Note that since this is the numerical integration discretisation
  // smaller n will provide coarser approximation to solution
  int n = 10000 ;
  vector<double> x = linspace(min_3sig,max_3sig,n) ;
  double dx = x[1]-x[0] ;
  double pImprove = 0.0 ;
  for (unsigned k = 0; k < x.size(); k++)
  {
    double p_cAi = 0.0 ;
    for (unsigned i = 0; i < ANodes.size(); i++)
    {
      double mu_Ai = ANodes[i]->GetMeanCTG() ;
      double sig_Ai = ANodes[i]->GetVarCTG() ;
      double p_cA1 = (1/(sig_Ai*sqrt(2*pi)))*exp(-(pow(x[k]-mu_Ai,2))/(2*pow(sig_Ai,2))) ;
      double p_cA2 = 1.0 ;
      for (unsigned j = 0; j < ANodes.size(); j++)
      {
        double mu_Aj = ANodes[j]->GetMeanCTG() ;
        double sig_Aj = ANodes[j]->GetVarCTG() ;
        if (j != i)
	        p_cA2 *= 0.5*erfc((x[k]-mu_Aj)/(sig_Aj*sqrt(2))) ;
      }
      p_cAi += p_cA1*p_cA2 ;
    }
    double p_cBi = 1.0 ;
    for (unsigned i = 0; i < BNodes.size(); i++)
    {
      double mu_Bi = BNodes[i]->GetMeanCTG() ;
      double sig_Bi = BNodes[i]->GetVarCTG() ;
      p_cBi *= 0.5*erfc((x[k]-(c_B0-c_A0)-mu_Bi)/(sig_Bi*sqrt(2))) ;
    }
    pImprove += (p_cAi)*(1-p_cBi)*dx ;
  }

  return (pImprove<=0.5);
}

void RAGS::SetInitialVert(Vertex * start)
{
  Vertex ** allVerts = itsGraph->GetVertices() ;
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
  {
    if ( start == allVerts[i] )
      itsVert = allVerts[i] ;
  }
}

XY RAGS::SearchGraph(XY start, XY goal, vector<double> &weights)
// Create function that converts from XY to Vertex *
{
	Vertex ** allVerts = itsGraph->GetVertices() ;
  Vertex * sVert ;
  Vertex * gVert ;
  bool sFound = false ;
  bool gFound = false ;
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
  {
    if ( start.x == allVerts[i]->GetX() && start.y == allVerts[i]->GetY() ){
      sVert = allVerts[i] ;
      sFound = true ;
    }
  }
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
  {
    if ( goal.x == allVerts[i]->GetX() && goal.y == allVerts[i]->GetY() ){
      gVert = allVerts[i] ;
      gFound = true ;
    }
  }
  if (!sFound)
  	printf("ERROR: start vertex (%f,%f) not found. Exiting.\n",start.x,start.y) ;
  if (!gFound)
  	printf("ERROR: goal vertex (%f,%f) not found. Exiting.\n",goal.x,goal.y) ;
	if (!sFound || !gFound)
		exit(1) ;
  return SearchGraph(sVert, gVert, weights) ;
}

XY RAGS::SearchGraph(Vertex * start, Vertex * goal, vector<double> &weights)
{
  AssignCurrentEdgeCosts(weights) ;
  
  // Initialise non-dominated path set
  if (itsNDSet.empty())
  {
    itsSearch = new Search(itsGraph, start, goal) ;
    vector<Node *> GSPaths = itsSearch->PathSearch(PSET) ;
    
    if (GSPaths.empty())
    {
      XY s = XY(start->GetX(),start->GetY()) ;
      return s ; // no paths found, stay where you are
    }
    
    for (ULONG i = 0; i < (ULONG)GSPaths.size(); i++)
      itsNDSet.push_back(GSPaths[i]->ReverseList(0)) ;
    for (ULONG i = 0; i < (ULONG)itsNDSet.size(); i++)
      itsNDSet[i]->SetCTG(GSPaths[i]->GetMeanCost(),GSPaths[i]->GetVarCost()) ;
    SetInitialVert(itsNDSet[0]->GetVertex()) ;
  }
    
  // Flag and return if current vertex does not match start vertex
  if (!(itsVert == start))
  {
    cout << "\nError: input start vertex (" << start->GetX() << "," << start->GetY() <<
    ") does not match stored vertex (" << itsVert->GetX() << "," << itsVert->GetY() << ")." <<
    endl ; 
    XY s = XY(start->GetX(),start->GetY()) ;
    return s ;
  }
  
  vector<Node *> newNodes = itsNDSet ;
  itsVert->SetNodes(newNodes) ;
  vector<Node *> tmpNodes ;
  vector<Vertex *> nextVerts ;
  
  // Identify next vertices
  for (unsigned i = 0; i < newNodes.size(); i++)
  {
	  bool newVert = true ;
	  for (unsigned j = 0; j < nextVerts.size(); j++)
	  {
		  if (nextVerts[j] == newNodes[i]->GetParent()->GetVertex() || nextVerts[j] == itsVert)
		  {
			  newVert = false ;
			  break ;
		  }
	  }
	  if (newVert)
		  nextVerts.push_back(newNodes[i]->GetParent()->GetVertex()) ;
  }
  
  // Identify next vertex path nodes
  for (unsigned i = 0; i < nextVerts.size(); i++)
  {
	  tmpNodes.clear() ;
	  for (unsigned j = 0; j < newNodes.size(); j++)
	  {
		  if (nextVerts[i] == newNodes[j]->GetParent()->GetVertex())
		  {
			  tmpNodes.push_back(newNodes[j]->GetParent()) ;
		  }
	  }
	  nextVerts[i]->SetNodes(tmpNodes) ;
  }
  
  // Rank next vertices according to probability of improvement
	sort(nextVerts.begin(),nextVerts.end(),ComputeImprovementProbability) ;
	
	itsVert = nextVerts[0] ;
	itsNDSet = itsVert->GetNodes() ;
	XY vertXY = XY(itsVert->GetX(),itsVert->GetY()) ;
	return vertXY ;
}

void RAGS::AssignCurrentEdgeCosts(vector<double> &weights)
{
  ULONG n = itsGraph->GetNumEdges() ;
  Edge ** e = itsGraph->GetEdges() ;
  
  for (ULONG i = 0; i < n; i++)
    e[i]->SetTrueCost(weights[i]) ;
}

int RAGS::GetEdgeIndex(XY start, XY goal)
{
	int startID ;
	int goalID ;
	bool foundStart = false ;
	bool foundGoal = false ;
	for (unsigned i = 0; i < itsLocations.size(); i++){
		if (start == itsLocations[i]){
			startID = i ;
			foundStart = true ;
		}
		if (goal == itsLocations[i]){
			goalID = i ;
			foundGoal = true ;
		}
		if (foundStart && foundGoal)
			break ;
	}
	if (!foundStart){
		cout << "ERROR: Did not find current vertex index.\n" ;
		exit(1) ;
	}
	if (!foundGoal){
		cout << "ERROR: Did not find next vertex index.\n" ;
		exit(1) ;
	}
	for (unsigned i = 0; i < itsEdgeArray.size(); i++){
		if (itsEdgeArray[i].first == startID && itsEdgeArray[i].second == goalID)
			return i ;
	}
	cout << "Error: Did not find current edge index. Exiting.\n" ;
	exit(1) ;
}
