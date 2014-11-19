#include "UndirectedGraph.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"
#include <string>

using namespace std;

/** Inserts an edge into graph
 * Uses an iterator to find the from vertex and checks if it is already in 
 * our map vertices, if not then it creates a new vertex and inserts it into
 * map. Same process repeated for to vertex. At the end those two vertexes
 * call the addEdge method defined in the Edge class
 */
void UndirectedGraph::addEdge(const std::string &from, const std::string &to,
            unsigned int cost, unsigned int length) {
  unordered_map<string, Vertex*>::const_iterator it = vertices.find(from);
  Vertex * fromVertex;
  Vertex * toVertex;
  if( it == vertices.end() ) {      // from vertex does not exist
    fromVertex = new Vertex( from );
    vertices.insert( make_pair(from, fromVertex ));
  }
  else {			    // from vertex exists
    fromVertex = it->second;
  }

  it = vertices.find( to );         
  if( it == vertices.end() ) {      // to vertex does not exist
    toVertex = new Vertex( to );
    vertices.insert( make_pair( to, toVertex ));
  }
  else {			    // to vertex exists
    toVertex = it->second;
  }
  //call Vertex.cpp addEdge to actually create/add the edge 
  fromVertex->addEdge( toVertex, cost, length );
  toVertex->addEdge( fromVertex, cost, length );
}

/**
 * Returns total cost of all edges in the graph
 * Since graph is undirected you divide cost by 2
 */
unsigned int UndirectedGraph::totalEdgeCost() const {
  int cost = 0;
  for ( auto it = vertices.begin(); it != vertices.end(); ++it )
    cost += it->second->totalEdgeCost();
  return cost/2;
  
}

/**
* Removes all edges from the graph except those necessary to
* form a minimum cost spanning tree of all vertices using Prim's
* algorithm.
*
* The graph must be in a state where such a spanning tree
* is possible. To call this method when a spanning tree is
* impossible is undefined behavior.
*/
void UndirectedGraph::minSpanningTree() {

}


/**
 * Determines the combined distance from the given Vertex to all
 * other Vertices in the graph using Dijkstra's algorithm.
 *
 * Returns max possible distance if the given Vertex does not appear
 * in the graph, or if any of the Vertices in the graph are not
 * reachable from the given Vertex. Otherwise, returns the combined
 * distance.
 */

unsigned int UndirectedGraph::totalDistance(const std::string &from) {
  priority_queue< std::pair<Vertex*, unsigned int>, 
                  vector<std::pair<Vertex*, unsigned int > >, 
		  DijkstraVertexComparator > pq;
 
  //enqueue the vertex that was passed in
  //find vertex in hashmap based on string passed in
  vToEnqueue = vertices[ from ];
  pairToEnqueue = make_pair( vToEnqueue, 0 );
  pq.push( pairToEnqueue );
  
  while(!pq.empty() ) {
  //	dequeue pair (v,c) from head thus removing the one with minimum cost
    std::pair<Vertex*, unsigned int> v = pq.pop();
  //	if( v->visited == true ) 	continue. Else, set it to true
 
    //if(v.first->visited == true )
    if( v.first.wasVisited() == true )
	continue;
    else
	//v.first->visited = true;
	v.first.setVisited( true );

    vector< Vertex* > unvisitedNeighbors = v.getUnvisitedNeighbors();

    vector< Vertex* >::const_iterator it = unvisitedNeighbors.begin();
    
//for( each of v's adjacent nodes (w) where visited == false
    while( it != unvisitedNeighbors.end() ) {

//Ideas from the class notes: 
    //NOTE: cost in D's alg is using latency aka length
  //calculate bestCost = v->cost + v->time;
    for( //dont know how to do this part )
      int score = //(v,w)'s edge cost + v's distance

//Tutor hints::
//vertex.getName() = string
//with name you can vertex edge
//from edge get length 
      if( score < //w's distance) {
      
      }  
  }
return 0;
}


/**
 * Determines the combined distance from all Vertices to all other
 * Vertices in the graph.
 *
 * Returns max possible distance if the graph is not connected.
 */
unsigned int UndirectedGraph::totalDistance() {
//loop through each vertex and call the other totalDistance on them
  return 0;
}


