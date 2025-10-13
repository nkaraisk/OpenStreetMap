//
// Created by nkara on 11/6/2025.
//

#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <vector>
#include <string>

#include "Vertex.hpp"
#include "tinyxml2.h"
#include "Utilities.hpp"

class Graph {
    private:
        std::vector<Vertex> vertexMap;
        std::unordered_map<unsigned long long, unsigned int> position;

    public:
        Graph();
        explicit Graph(const std::string &filename);

        [[nodiscard]] std::vector<Vertex> &getVertexMap() {
            return vertexMap;
        }

        [[nodiscard]] std::unordered_map<unsigned long long, unsigned int> &getPosition() {
            return position;
        }

        bool addVertex(unsigned long long id, double lat, double lon);
        bool deleteVertex(unsigned long long id);

        std::list<Edge> shortestPath(unsigned long long source, unsigned long long destination);

        // std::list<Vertex> dijkstraShortestPath(unsigned long source, unsigned long destination);

        void graphCompression();

        std::list<unsigned long long> BFS(unsigned long long start);

        std::list<unsigned long long> DFS(unsigned long long start);
};



#endif //GRAPH_H