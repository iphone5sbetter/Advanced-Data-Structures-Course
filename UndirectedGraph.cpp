#include "UndirectedGraph.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"

void UndirectedGraph::addEdge(const std::string &from, const std::string &to,
            unsigned int cost, unsigned int length) {

}

unsigned int UndirectedGraph::totalEdgeCost() const {
  int cost = 0;
  for ( auto it = vertices.begin(); it != vertices.end(); ++it )
    cost += it->second->totalEdgeCost();
  return cost/2;
  
}

void UndirectedGraph::minSpanningTree() {

}

unsigned int UndirectedGraph::totalDistance(const std::string &from) {
//D's algorithm
}

unsigned int UndirectedGraph::totalDistance() {
//loop through each vertex and call the other totalDistance on them
}
