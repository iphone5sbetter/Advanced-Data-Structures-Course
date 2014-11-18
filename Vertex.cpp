#include "Vertex.hpp"
#include "UndirectedGraph.hpp"
#include <string>
using namespace std;

Vertex::Vertex ( const std::string &name ) {
  this->name = name;
  distance = -1; 
  visited = false;
}

bool Vertex::addEdge(Vertex *to, unsigned int cost, unsigned int length) {
  Edge edge = Edge(this, to, cost, length); // Create new edge
   
  //
  edges[to->name] = edge;

  //We need to check if this edge already exists here before inserting
  //it into the map edges...if it doesnt then insert
  return edges.insert(std::make_pair(to->getName(), edge)).second; 
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
  unordered_map<string, Vertex*>::const_iterator it = vertices.find(this->getName());
  it->second->visited = visited;
  /** if (!(edges.find(name)))	// Not visited
    visited = false;
  }
  else {			// Visited
    visited = true;
  } */
}

void clearEdges() {

}

unsigned int Vertex::totalEdgeCost() const {
  int cost = 0;
  for ( auto it = edges.begin(); it != edges.end(); ++it )
    cost += it->second.getCost();
  return cost;
}

