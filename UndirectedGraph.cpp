#include "UndirectedGraph.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"
#include <string>
#include <queue>
#include <math.h>

#include <cstdlib>
#include <fstream>
#include <iostream>

//const int INFINITY = 2147483647;
//const int INFINITY = INT_MAX;


using namespace std;

/**void UndirectedGraph::addEdge(const std::string &from, const std::string &to,
            unsigned int cost, unsigned int length) 
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
  //create a new graph and priority queue
  UndirectedGraph graph;
  std::priority_queue< Edge, std::vector<Edge>, MSTComparator > pq;

  auto s = vertices.begin();
  while (s != vertices.end()) {
    s->second->setVisited(false);
    s++;
  }

  Vertex * arbitrary = vertices.begin()->second;	// Arbitrary vertex

  arbitrary->setVisited(true);

  // Iterate through adjacency list (go through all edges in hash map "edges")
  unordered_map<std::string, Edge>::iterator it = arbitrary->edges.begin();
  for (; it != arbitrary->edges.end(); it++) {
    pq.push(it->second);		// Put edges in queue
  }
  
  //iterator to go through the edges
  unordered_map<std::string, Edge>::iterator itEdge;

  while(!pq.empty()) {
    Edge e = pq.top();
    pq.pop();			// Remove edge with smallest cost
    if (e.getTo()->wasVisited() == true) {
      continue;
    }
    else {
      e.getTo()->setVisited(true);
      graph.addEdge(e.getFrom()->getName(), e.getTo()->getName(), e.getCost(),
								 e.getLength());

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
  return graph;
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
  //make sure that all distances are infinity to begin and visited is false
  auto it = vertices.begin();
  while ( it!= vertices.end() ) {
    it->second->setDistance( INFINITY );
    it->second->setVisited( false );
    it++;
  }

  //create a priority queue
  std::priority_queue< std::pair<Vertex*, unsigned int>, 
                  std::vector<std::pair<Vertex*, unsigned int > >, 
		  DijkstraVertexComparator > pq;

  //enqueue the vertex that was passed in
  //find vertex in hashmap based on string passed in
  Vertex * vToEnqueue = vertices[ from ];



  //set source vertex distance to zero
  vToEnqueue->setDistance( 0 );

  std::pair<Vertex*, unsigned int> pairToEnqueue = 
		std::make_pair( vToEnqueue, vToEnqueue->getDistance() );

  std::make_pair( vToEnqueue, 0 );

  pq.push( pairToEnqueue );


  while( !pq.empty() ) {
    //	dequeue pair (v,c) from head thus removing the one with minimum cost
    std::pair<Vertex*, unsigned int> v = pq.top();
    pq.pop();

    //	if( v->visited == true ) 	continue. Else, set it to true
    if( v.first->wasVisited() == true )
	continue;
    else
	//v.first->visited = true;
	v.first->setVisited( true );


    std::vector< std::pair< Vertex*, Edge> > unvisitedNeighbors = 
				v.first->getUnvisitedNeighbors();


    std::vector<std::pair<Vertex*, Edge> >::const_iterator it = 
						unvisitedNeighbors.begin();

    
    //for( each of v's adjacent nodes (w) where visited == false
    while( it != unvisitedNeighbors.end() ) {

      //get the edge
      Edge vwEdge = it->second;

      //get the "w" vertex aka an adjacent node that has been unvisited
      Vertex * wVertex = it->first;


      //calculate score
      unsigned int score = vwEdge.getLength() + v.first->getDistance();

      //if score is less than w's distance set w's distance to score 
      if( score < wVertex->getDistance() ) {
         wVertex->setDistance( score );


      }
      //enqueue w
      pq.push( std::make_pair( wVertex, wVertex->getDistance() ) );
      //go to next unvisited neighbor
      it++;  
      //v.first->setVisited(true);
    }
  }

  //create an iterator to iterate through vertices map
  auto vertices_it = vertices.begin();

  unsigned int toReturn = 0;

  while ( vertices_it != vertices.end() ){

    //sum all of the distances
    toReturn += vertices_it->second->getDistance();
    //go to next vertex
    vertices_it++;
  }

  return toReturn;
}


/**
 * Determines the combined distance from all Vertices to all other
 * Vertices in the graph.
 *
 * Returns max possible distance if the graph is not connected.
 */
unsigned int UndirectedGraph::totalDistance() {
  //loop through each vertex and call the other totalDistance on them

  //create iterator for vertices
  auto it = vertices.begin();
  unsigned int totalDistance = 0;


  //loop through all of vertices
  while( it != vertices.end() ) {


  //	sumTotalDistance += call totalDistance for all of them


    totalDistance += this->totalDistance(it->first);
    it++;



  }

  return totalDistance;
}


