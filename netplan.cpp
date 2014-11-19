#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

#include "UndirectedGraph.hpp"

using namespace std;

/**
 * Entry point into the netplan program.
 *
 * -Reads a file from the filesystem according to the specification for
 *  PA3, creating an UndirectedGraph.
 * -Finds the total cost & ping time of the graph as presented in the input
 *  file.
 * -Determines the minimum cost graph from the original graph.
 * -Finds the total cost & ping time of the minimum cost graph.
 * -Finds the change of cost & ping time from the original graph to the
 *  minimum cost graph.
 * -Prints the results to stdout.
 *
 * Usage:
 *   ./netplan infile
 *
 */
int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " infile" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "Unable to open file for reading." << std::endl;
        return EXIT_FAILURE;
    }
    
    UndirectedGraph g = UndirectedGraph();
    std::string v1;
    std::string v2;
    int cost;
    int time;
    while (in.good()) {
      in >> v1 >> v2 >> cost >> time;
      if(!in.good()) 
        break; 
 //add vertices
 //add edge
     g.addEdge( v1, v2, cost, time );
     }

     cout<< g.totalEdgeCost()<< endl;
     cout<< "Part 2 not yet implemented" << endl;
     cout<< "Part 3 not yet implemented" << endl;
    return EXIT_SUCCESS;
}
