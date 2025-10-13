//
// Created by nkara on 11/6/2025.
//

#ifndef UTILITIES_H
#define UTILITIES_H

#include "Vertex.hpp"
#include "tinyxml2.h"

#include <unordered_map>
#include <string>
#include <vector>
#include <queue>

class Utilities {
    public:
        //Distance Calculation Methods
        static double degToRad(double degrees);

        static double halfDifference(double p1, double p2);

        static double aCalculation(Vertex const &v1, Vertex const &v2);

        static double cFactorCalculation(Vertex const &v1, Vertex const &v2);

        static double haversineDistance(Vertex const &v1, Vertex const &v2);

        //Read XML Methods
        static void createVertexList(tinyxml2::XMLElement* root, std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position);

        static double checkHighwayFactor(const char* highway);

        static double checkHighway(tinyxml2::XMLElement* way);

        static bool isOneWay(tinyxml2::XMLElement* way);

        static void readRefs(tinyxml2::XMLElement* way, std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position);

        static void removeBlankNodes(std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position);

        static void createNodeEdge(tinyxml2::XMLElement* root, std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position);

        //Helpers
        static Vertex* findVertex(unsigned long long vertexId, std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position);

        //Nodes Handling
        static void EdgeDeletion(Vertex &deletingVertex, std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position);

        static void mapUpdates(unsigned int numPos, const std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position);

        //Dijkstra's helpers
        struct CompareBySecond {
            bool operator() (const std::pair<unsigned long long,double>& a, const std::pair<unsigned long long,double>& b) const {
                return a.second < b.second || (a.second == b.second && a.first < b.first);
            }
        };

        static void dijkstraIterative(std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position, std::unordered_map<unsigned long long, std::tuple<unsigned long long, double>> &data, const std::pair<unsigned long long, double> &startPair);

        // private:
        //     static void PairsUpdate(std::set<std::pair<unsigned long long,double>, CompareBySecond> &toVisit, const std::pair<unsigned long long ,double> &currentPair, Vertex &currentVertex);
        //
        // public:
        //     static void dijkstraRecursion(std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned> &position, std::unordered_map<unsigned long long, std::tuple<unsigned long long, double>> &data, std::set<std::pair<unsigned long long,double>, CompareBySecond> &toVisit, std::vector<bool> &visited, std::pair<unsigned long long, double> &currentPair);

        //Merge function
        static void merging(std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position, int mode, const Edge &start, Vertex &mid, const Edge &end);

        //BFS function
        static void bfsStructure(std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position, std::queue<Vertex*> &FIFO, Vertex *startingPoint);

        //DFS function
        static void dfsStructure(std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position, Vertex *startingPoint, std::list<unsigned long long> &result);

        //Printing methods
        static void printPath(std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position, std::list<Edge> &path);

        static void printFS(const std::list<unsigned long long> &result);
};


#endif //UTILITIES_H