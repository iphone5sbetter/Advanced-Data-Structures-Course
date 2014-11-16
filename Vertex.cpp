#include "Vertex.hpp"

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

const Vertex::std::string &getName() const {
  return name;
}

unsigned int Vertex::getDistance() const {
  return distance;
}

void Vertex::setDistance(unsigned int distance) {
  this->distance = distance;
}

bool Vertex::wasVisited() const {
  return if(this->visited);
}

void Vertex::setVisited(bool visited) {
  if (!(edges->find(to->name)))
    return true;
  }
  else {
    return false;
  }
}

void clearEdges() {

}

unsigned int void::totalEdgeCost() const {
  int cost = 0;
  for ( auto it = edges.begin(); it != edges.end(); ++it )
    cost += it->second.getCost();
  return cost;
}

