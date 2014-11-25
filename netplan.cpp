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
    bool emptyGraph = true;
    while (in.good()) {
      //read input
      in >> v1 >> v2 >> cost >> time;

      if(!in.good()) {
        if( emptyGraph ) {
          //print zeros
          for( int i = 0; i < 6 ; i++ ) 
            cout<< "0" << endl;
          //end
          return 0;
        }
        break; 
      }
      //if you made it past the very first iteration of checking if input is 
      //there than you wont have an empty graph so set the bool to false
      emptyGraph = false;

      //update the graph with the info on the current line
      g.addEdge( v1, v2, cost, time );
    }


    UndirectedGraph primsGraph = g.minSpanningTree();
     

     cout<< g.totalEdgeCost()<< endl;
     cout<< primsGraph.totalEdgeCost() << endl;
     cout<< g.totalEdgeCost() - primsGraph.totalEdgeCost() << endl;
     cout<< g.totalDistance() << endl;
     cout<< primsGraph.totalDistance() << endl;
     cout<< primsGraph.totalDistance() - g.totalDistance() << endl;
    return EXIT_SUCCESS;
}
