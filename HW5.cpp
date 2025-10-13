#include <iostream>
#include <sstream>

#include "Graph.hpp"
#include "Utilities.hpp"

using namespace std;


int main() {
    string mode;
    string input;
    string sidStr;
    string eidStr;

    Graph myGraph;

    do {
        unsigned long long sid = 0;
        unsigned long long eid = 0;

        // cout << endl << "-i <filepath>  :   Import Graph from <filepath>" << endl;
        // cout << "-c             :   Compact Graph" << endl;
        // cout << "-p <sid> <eid> :   Estimate the shortest path between start" << endl;
        // cout << "                   node with <sid> and end node with <eid>" << endl;
        // cout << "-b <sid>       :   Print bfs starting from node with <sid>" << endl;
        // cout << "-d <sid>       :   Print dfs starting from node with <sid>" << endl;
        // cout << "-q             :   Exit without memory leaks" << endl << endl;

        cout << "\n" << "Enter your choice:  ";

        //Input
        getline(cin, input);

        stringstream ss(input);
        ss >> mode;

        //First choice(-i)
        if (mode == "-i") {
            string filename;
            myGraph = Graph();

            if (!(ss >> filename)) {
                cout << "\n" << "No filename given" << endl;
                continue;
            }

            myGraph = Graph(filename);
            if (!myGraph.getVertexMap().empty()) {
                cout << "\n" << "Graph OK" << endl;
            }
        }
        // //

        //Second choice(-c)
        else if (mode == "-c") {
            myGraph.graphCompression();
            cout << "\n" << "Compact OK" << endl;
        }
        // //

        //Third choice(-p)
        else if (mode == "-p") {
            if (!(ss >> sidStr >> eidStr)) {
                cout << "Missing sid or eid" << endl;
                continue;
            }

            sid = strtoull(sidStr.c_str(), nullptr, 10);
            eid = strtoull(eidStr.c_str(), nullptr, 10);

            list<Edge> path = myGraph.shortestPath(sid, eid);

            Utilities::printPath(myGraph.getVertexMap(), myGraph.getPosition(), path);
        }
        // //

        //Fourth choice(-b)
        else if (mode == "-b") {
            if (!(ss >> sidStr)) {
                cout << "Missing sid" << endl;
                continue;
            }
            sid = strtoull(sidStr.c_str(), nullptr, 10);

            list<unsigned long long> idList = myGraph.BFS(sid);

            Utilities::printFS(idList);
        }
        // //

        //Fifth choice(-d)
        else if (mode == "-d") {
            if (!(ss >> sidStr)) {
                cout << "Missing sid" << endl;
                continue;
            }
            sid = strtoull(sidStr.c_str(), nullptr, 10);

            list<unsigned long long> idList = myGraph.DFS(sid);

            Utilities::printFS(idList);
        }
        //

        //Sixth choice(-q)
        else if (mode == "-q") {
            break;
        }
        //

        //Any other choice
        else {
            cout << "Invalid input" << endl;
        }
        //

    }while (mode != "-q");
}