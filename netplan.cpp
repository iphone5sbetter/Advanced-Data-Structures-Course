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
    if (argc != 2) {		// Incorrect number of args
        std::cerr << "Usage: " << argv[0] << " infile" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::ifstream in(argv[1]);  // Unreadable file
    if (!in) {
        std::cerr << "Unable to open file for reading." << std::endl;
        return EXIT_FAILURE;
    }
    
    UndirectedGraph g = UndirectedGraph(); // Create new graph
    std::string v1;			   // String for first arg
    std::string v2;			   // String for second arg
    int cost;				   // Cost
    int time;				   // Time, latency
    bool emptyGraph = true;		   // Flag for empty file/graph
    while (in.good()) {			   // File contains good stream
      					   // Read input
      in >> v1 >> v2 >> cost >> time;	   // Set user's input to variables

      if(!in.good()) {			   // File does not contain good stream
        if( emptyGraph ) {		   // Empty graph case
          //print zeros
          for( int i = 0; i < 6 ; i++ )    
            cout<< "0" << endl;		   // Output of 0's
          //end
          return 0;			   // Exit
        }
        break; 
      }
      // If you made it past the very first iteration of checking if input is 
      // there than you wont have an empty graph so set the bool to false
      emptyGraph = false;

      // Update the graph with the info on the current line to add edge
      g.addEdge( v1, v2, cost, time );
    }

    // Build minimum spanning tree
    UndirectedGraph primsGraph = g.minSpanningTree();
     
     // Total cost of building all possible network links
     cout<< g.totalEdgeCost()<< endl;
     // Total cost of building cheapest network
     cout<< primsGraph.totalEdgeCost() << endl;
     // Money saved by building minimum-cost network
     cout<< g.totalEdgeCost() - primsGraph.totalEdgeCost() << endl;
     // Total transit time to send a packet between all pairs of computers
     cout<< g.totalDistance() << endl;
     // Total transit time to send a packet between all pairs of computers
     // in minimum-cost network
     cout<< primsGraph.totalDistance() << endl;
     // Increase in "total time" required for packet travel in cheap network
     cout<< primsGraph.totalDistance() - g.totalDistance() << endl;
    return EXIT_SUCCESS;
}
