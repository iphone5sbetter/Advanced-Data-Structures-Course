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
   
  edges[to->name] = edge;


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
  this->visited = visited;
}

void clearEdges() {

}

unsigned int Vertex::totalEdgeCost() const {
  int cost = 0;
  for ( auto it = edges.begin(); it != edges.end(); ++it )
    cost += it->second.getCost();
  return cost;
}

vector< Vertex* >& Vertex::getUnvisitedNeighbors() const {
  //create an iterator and set it to the beginning of your edge list
  unordered_map<std::string, Edge>::const_iterator it = edges.begin();

  //create a vector list to store the vertices that are neighbors
  vector< Vertex* > neighbors;

  while( it != edges.end() ) {

    auto edge = it->second;
    Vertex* currentNeighbor = edge.getTo();
    if( currentNeighbor->wasVisited() == false ) {
      neighbors.push_back( currentNeighbor );
    }
    //increment the iterator to the next value in edges()
    it++;
  }

  return neighbors;
  
}
  
   
}
