#include "Edge.hpp"

using namespace std;

Edge::Edge() {
}

Edge::Edge(Vertex *from, Vertex *to, unsigned int cost, unsigned int length) {
  this->from = from;			// Assign starting vertex
  this->to = to;			// Assign ending vertex
  this->cost = cost;			// Assign cost of edge
  this->length = length;		// Assign length of edge
}

Vertex* Edge::getFrom() const {
  return this->from;			// Obtain starting vertex of edge
}

Vertex* Edge::getTo() const {
  return this->to;			// Obtain ending vertex of edge
}

void Edge::setCost(unsigned int cost) {
  this->cost = cost;			// Assign cost of edge
}

unsigned int Edge::getCost () const {
  return cost;				// Obtain cost of edge
}

void Edge::setLength(unsigned int length) {
  this->length = length;		// Assign length of edge
}

unsigned int Edge::getLength() const {
  return length;			// Obtain length of edge
}

bool Edge::operator<(const Edge &right) const {
  return cost < right.cost;		// Sorts cost in increasing order in pq
}

