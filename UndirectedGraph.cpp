#include "UndirectedGraph.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"
#include <string>
#include <queue>
#include <math.h>

#include <cstdlib>
#include <fstream>
#include <iostream>


using namespace std;

/**void UndirectedGraph::addEdge(const std::string &from,             
 * const std::string &to, unsigned int cost, unsigned int length) 
 * Inserts an edge into graph
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
  if( it == vertices.end()) {      // From vertex does not exist
    fromVertex = new Vertex( from );
    vertices.insert( make_pair(from, fromVertex ));
  }
  else {			    // From vertex exists
    fromVertex = it->second;
  }

  it = vertices.find( to );         
  if( it == vertices.end() ) {      // To vertex does not exist
    toVertex = new Vertex( to );
    vertices.insert( make_pair( to, toVertex ));
  }
  else {			    // To vertex exists
    toVertex = it->second;
  }
  //Call Vertex.cpp addEdge to actually create/add the edge 
  fromVertex->addEdge( toVertex, cost, length );
  toVertex->addEdge( fromVertex, cost, length );
}


/** unsigned int UndirectedGraph::totalEdgeCost() const 
 * Returns total cost of all edges in the graph
 * Since graph is undirected you divide cost by 2
 */
unsigned int UndirectedGraph::totalEdgeCost() const {
  int cost = 0;
  for ( auto it = vertices.begin(); it != vertices.end(); ++it )
    cost += it->second->totalEdgeCost();
  return cost/2;
  
}


/** UndirectedGraph UndirectedGraph::minSpanningTree() 
 * Removes all edges from the graph except those necessary to
 * form a minimum cost spanning tree of all vertices using Prim's
 * algorithm.
 *
 * The graph must be in a state where such a spanning tree
 * is possible. To call this method when a spanning tree is
 * impossible is undefined behavior.
 */
UndirectedGraph UndirectedGraph::minSpanningTree() {
  // Create a new graph and priority queue
  UndirectedGraph graph;
  std::priority_queue< Edge, std::vector<Edge>, MSTComparator > pq;

  //Iterator for vertices
  auto s = vertices.begin();

  // Set all vertices to false
  while (s != vertices.end()) {
    s->second->setVisited(false);
    s++;
  }
  
  Vertex * arbitrary = vertices.begin()->second;	// Arbitrary vertex

  arbitrary->setVisited(true);				// Set to true

  // Iterate through adjacency list (go through all edges in hash map "edges")
  unordered_map<std::string, Edge>::iterator it = arbitrary->edges.begin();
  for (; it != arbitrary->edges.end(); it++) {
    pq.push(it->second);		// Put edges in queue
  }
  
  // Iterator to go through the edges
  unordered_map<std::string, Edge>::iterator itEdge;

  while(!pq.empty()) {
    Edge e = pq.top();
    pq.pop();			// Remove edge with smallest cost
    // If this edge was already added to MST continue
    if (e.getTo()->wasVisited() == true) {
      continue;
    }
    else {
    // Else create the edge and flag it to be visited
      e.getTo()->setVisited(true);
      graph.addEdge(e.getFrom()->getName(), e.getTo()->getName(), e.getCost(),
							 e.getLength());

      // Go through all of this vertex's edges for MST growing algorithm
      for (itEdge = e.getTo()->edges.begin(); itEdge != e.getTo()->edges.end();
								 itEdge++)  
      {
        Vertex * vertex = itEdge->second.getTo();
        if (vertex->wasVisited() == false) {	
	  pq.push(itEdge->second);
	}
      }
    }
  } 
  // Return your MST
  return graph;
}


/** unsigned int UndirectedGraph::totalDistance(const std::string &from)
 * Determines the combined distance from the given Vertex to all
 * other Vertices in the graph using Dijkstra's algorithm.
 *
 * Returns max possible distance if the given Vertex does not appear
 * in the graph, or if any of the Vertices in the graph are not
 * reachable from the given Vertex. Otherwise, returns the combined
 * distance.
 */

unsigned int UndirectedGraph::totalDistance(const std::string &from) {
  //make sure that all distances are infinity to begin and visited is false
  auto it = vertices.begin();
  while ( it!= vertices.end() ) {
    it->second->setDistance( INFINITY );
    it->second->setVisited( false );
    it++;
  }

  // Create a priority queue
  std::priority_queue< std::pair<Vertex*, unsigned int>, 
                  std::vector<std::pair<Vertex*, unsigned int > >, 
		  DijkstraVertexComparator > pq;

  // Enqueue the vertex that was passed in
  // Find vertex in hashmap based on string passed in
  Vertex * vToEnqueue = vertices[ from ];



  // Set source vertex distance to zero
  vToEnqueue->setDistance( 0 );

  // Push the pair to the priority queue
  std::pair<Vertex*, unsigned int> pairToEnqueue = 
		std::make_pair( vToEnqueue, vToEnqueue->getDistance() );

  std::make_pair( vToEnqueue, 0 );

  pq.push( pairToEnqueue );


  while( !pq.empty() ) {
    // Dequeue pair (v,c) from head thus removing the one with minimum cost
    std::pair<Vertex*, unsigned int> v = pq.top();
    pq.pop();

    // If( v->visited == true ) continue. Else, set it to true
    if( v.first->wasVisited() == true )
	continue;
    else
	//v.first->visited = true;
	v.first->setVisited( true );


    std::vector< std::pair< Vertex*, Edge> > unvisitedNeighbors = 
				v.first->getUnvisitedNeighbors();


    std::vector<std::pair<Vertex*, Edge> >::const_iterator it = 
						unvisitedNeighbors.begin();

    
    // For each of v's adjacent nodes (w) where visited == false
    while( it != unvisitedNeighbors.end() ) {

      // Get the edge
      Edge vwEdge = it->second;

      // Get the "w" vertex aka an adjacent node that has been unvisited
      Vertex * wVertex = it->first;


      // Calculate score
      unsigned int score = vwEdge.getLength() + v.first->getDistance();

      // If score is less than w's distance set w's distance to score 
      if( score < wVertex->getDistance() ) {
         wVertex->setDistance( score );


      }
      // Enqueue w
      pq.push( std::make_pair( wVertex, wVertex->getDistance() ) );
      // Go to next unvisited neighbor
      it++;  
    }
  }

  // Create an iterator to iterate through vertices map
  auto vertices_it = vertices.begin();

  unsigned int toReturn = 0;

  while ( vertices_it != vertices.end() ){

    // Sum all of the distances
    toReturn += vertices_it->second->getDistance();
    // Go to next vertex
    vertices_it++;
  }

  return toReturn;
}


/** unsigned int UndirectedGraph::totalDistance() 
 * Determines the combined distance from all Vertices to all other
 * Vertices in the graph.
 *
 * Returns max possible distance if the graph is not connected.
 */
unsigned int UndirectedGraph::totalDistance() {
  // Loop through each vertex and call the other totalDistance on them

  // Create iterator for vertices
  auto it = vertices.begin();
  unsigned int totalDistance = 0;


  // Loop through all of vertices
  while( it != vertices.end() ) {
    // Call the other totalDistance method on each vertex and sum up all the 
    // results
    totalDistance += this->totalDistance(it->first);
    it++;
  }
  return totalDistance;
}



/**
 * Destructs an UndirectedGraph.
 */
UndirectedGraph::~UndirectedGraph(){
  // Goes through all the vertices and calls clearEdges() for each one of them
  for( auto it = this->vertices.begin(); it != vertices.end(); it++ ) {
    it->second->clearEdges();
    delete it->second;
  }
  // Clear the vertices
  vertices.clear();
}
