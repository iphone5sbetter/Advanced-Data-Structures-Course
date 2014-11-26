#include "Vertex.hpp"
#include "UndirectedGraph.hpp"
#include <string>
#include <queue>
#include <math.h>

using namespace std;

Vertex::Vertex ( const std::string &name ) {
  this->name = name;			// Set name of vertex
  distance = INFINITY; 			// Set distance of vertex
  visited = false;			// Vertex is unvisited
}

void Vertex::addEdge(Vertex *to, unsigned int cost, unsigned int length) {
  Edge edge = Edge(this, to, cost, length); // Create new edge
  edges.insert(std::make_pair(to->name, edge)); // Insert new edge
}

const std::string& Vertex::getName() const {
  return name;				// Obtain name of vertex
}

unsigned int Vertex::getDistance() const {
  return distance;			// Obtain distance of vertex
}

void Vertex::setDistance(unsigned int distance) {
  this->distance = distance;		// Set distance of vertex
}

bool Vertex::wasVisited() const {
  return this->visited;			// Determine if vertex has been visited
}

void Vertex::setVisited(bool visited) {
  this->visited = visited;		// Set visited state
}

void Vertex::clearEdges() {
  // Clear out the edges map for the vertex
 // for( auto it = this->edges.begin(); it != edges.end(); it++ ) {
    
    edges.clear();
  //}
}

unsigned int Vertex::totalEdgeCost() const {
  int cost = 0;
  // Iterate through edges
  for ( auto it = edges.begin(); it != edges.end(); ++it )
    cost += it->second.getCost();	// Determine total cost
  return cost;
}

std::vector< std::pair<Vertex*, Edge> >& Vertex::getUnvisitedNeighbors() const 
{
  // Create an iterator and set it to the beginning of your edge list
  unordered_map<std::string, Edge>::const_iterator it = edges.begin();

  // Create a vector list to store the vertices that are neighbors
  std::vector< std::pair<Vertex*, Edge> > * neighbors 
			= new std::vector< std::pair<Vertex*, Edge> >();

  while( it != edges.end() ) {		// Iterate through all edges

    auto edge = it->second;		// Obtain edge
    Vertex* currentNeighbor = edge.getTo(); // Vertex pointer to end point
    if( currentNeighbor->wasVisited() == false ) {
      // Add neighboring vertices to vector 
      neighbors->push_back( std::make_pair(currentNeighbor, edge) );
    }
    // Increment the iterator to the next value in edges()
    it++;
  }

  return *neighbors;			// Vector of neighboring vertices
  
}

  
   

