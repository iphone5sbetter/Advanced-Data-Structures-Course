#include "UndirectedGraph.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"
#include <string>

using namespace std;

void UndirectedGraph::addEdge(const std::string &from, const std::string &to,
            unsigned int cost, unsigned int length) {
/////////////////////////////////////////////////////////////////////////
//Where do we use the addEdge in vertex?? Do we call it for to and for from?
/////////////////////////////////////////////////////////////////////////
  unordered_map<string, Edge>::const_iterator it = vertices.find(from);
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
  fromVertex.addEdge( toVertex, cost, length );
  toVertex.addEdge( fromVertex, cost, length );
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
