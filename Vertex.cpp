#include "Vertex.hpp"
#include "UndirectedGraph.hpp"
#include <string>
#include <queue>
#include <math.h>

using namespace std;

Vertex::Vertex ( const std::string &name ) {
  this->name = name;
  distance = INFINITY; 
  visited = false;
}

void Vertex::addEdge(Vertex *to, unsigned int cost, unsigned int length) {
  Edge edge = Edge(this, to, cost, length); // Create new edge
   
//  edges[to->name] = edge;
  edges.insert(std::make_pair(to->name, edge));


//  return edges.insert(std::make_pair(to->getName(), edge)).second; 
}

const std::string& Vertex::getName() const {
  return name;
}

unsigned int Vertex::getDistance() const {
  return distance;
}

void Vertex::setDistance(unsigned int distance) {
  this->distance = distance;
}

bool Vertex::wasVisited() const {
  return this->visited;
}

void Vertex::setVisited(bool visited) {
  this->visited = visited;
}

void clearEdges() {
  //clear out the edges map for the vertex
  auto edges_it = edges.begin();
  while ( edges_it != edges.end() ) {
    edges_it.clear();
    edges_it++;
  }
}

unsigned int Vertex::totalEdgeCost() const {
  int cost = 0;
  for ( auto it = edges.begin(); it != edges.end(); ++it )
    cost += it->second.getCost();
  return cost;
}

std::vector< std::pair<Vertex*, Edge> >& Vertex::getUnvisitedNeighbors() const {
  //create an iterator and set it to the beginning of your edge list
  unordered_map<std::string, Edge>::const_iterator it = edges.begin();

  //create a vector list to store the vertices that are neighbors
  std::vector< std::pair<Vertex*, Edge> > * neighbors 
			= new std::vector< std::pair<Vertex*, Edge> >();

  while( it != edges.end() ) {

    auto edge = it->second;
    Vertex* currentNeighbor = edge.getTo();
    if( currentNeighbor->wasVisited() == false ) {
      neighbors->push_back( std::make_pair(currentNeighbor, edge) );
    }
    //increment the iterator to the next value in edges()
    it++;
  }

  return *neighbors;
  
}

  
   

