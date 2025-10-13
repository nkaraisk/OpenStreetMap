//
// Created by nkara on 11/6/2025.
//

#include "Utilities.hpp"

#include <set>
#include <queue>
#include <cmath>
#include <list>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <stack>
#include <unordered_set>

using namespace tinyxml2;
using namespace std;

//Distance Calculation Methods
double Utilities::degToRad(const double degrees) {
    return degrees * (M_PI / 180);
}


double Utilities::halfDifference(const double p1, const double p2) {
    double diff = p1 - p2;
    return diff/2;
}


double Utilities::aCalculation(Vertex const &v1, Vertex const &v2) {
    double sin1 = sin(halfDifference(degToRad(v1.getLatitude()), degToRad(v2.getLatitude())));
    double sin2 = sin(halfDifference(degToRad(v1.getLongitude()), degToRad(v2.getLongitude())));
    double cos1 = cos(degToRad(v1.getLatitude()));
    double cos2 = cos(degToRad(v2.getLatitude()));

    return (sin1*sin1) + cos1 * cos2 * (sin2*sin2);
}

double Utilities::cFactorCalculation(Vertex const &v1, Vertex const &v2) {
    double a = aCalculation(v1, v2);
    double aSqrt = sqrt(a);
    double a_1Sqrt = sqrt(1-a);

    return 2 * atan2(aSqrt, a_1Sqrt);
}

double Utilities::haversineDistance(Vertex const &v1, Vertex const &v2) {
    constexpr double R = 6378137;

    return R * cFactorCalculation(v1, v2);
}


//Read XML Methods

//This method creates all the nodes that the xml file contains along with their position inside the vector.
void Utilities::createVertexList(XMLElement* root, vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position) {

    int pos = 0;
    for (XMLElement* node = root->FirstChildElement("node"); node != nullptr; node = node->NextSiblingElement("node"), pos++) {
        const char* idAttr = node->Attribute("id");
        unsigned long long id = strtoull(idAttr, nullptr, 10);
        myVector.emplace_back(id, node->DoubleAttribute("lat"), node->DoubleAttribute("lon"));

        position.emplace(id, pos);
    }
}


//Translates the highway's factor string to its value.
double Utilities::checkHighwayFactor(const char* highway) {
    if (!strcmp(highway, "motorway") || !strcmp(highway, "trunk")) {
        return 0.5;
    }

    if (!strcmp(highway, "primary") || !strcmp(highway, "secondary")) {
        return 0.75;
    }

    if (!strcmp(highway, "tertiary") || !strcmp(highway, "residential")) {
        return 1.0;
    }

    if (!strcmp(highway, "living_street") || !strcmp(highway, "unclassified")) {
        return 1.25;
    }

    if (!strcmp(highway, "service") || !strcmp(highway, "track")) {
        return 1.5;
    }

    //If there is another value for "highway" we return 1.
    return 1;
}


//Finds the highway string inside the xml file
double Utilities::checkHighway(XMLElement* way) {
    for (XMLElement* tag = way->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
        const char* k = tag->Attribute("k");
        const char* v = tag->Attribute("v");

        if (k && v && string(k) == "highway") {
            return checkHighwayFactor(v);
        }
    }

    return 0;
}


bool Utilities::isOneWay(XMLElement* way) {
    for (XMLElement* tag = way->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
        const char* k = tag->Attribute("k");
        const char* v = tag->Attribute("v");

        if (k && v && (strcmp(k, "oneway") == 0)) {
            string val = string(v);
            return (val == "yes" || val == "1");
        }
    }

    return false;
}


Vertex* Utilities::findVertex(const unsigned long long vertexId, vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position) {
    const auto positionIndex = position.find(vertexId);
    if (positionIndex == position.end()) {
        return nullptr;
    }

    return &myVector[positionIndex->second];
}


void Utilities::readRefs(XMLElement *way, vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position) {
    const bool oneWay = isOneWay(way);
    const double factor = checkHighway(way);

    if (factor <= 0) {
        return;
    }

    //Finding the first Starting node.
    XMLElement* nd = way->FirstChildElement("nd");
    if (!nd) {
        return;
    }
    const char* refStartingID = nd->Attribute("ref");
    const unsigned long long startingID = strtoull(refStartingID, nullptr, 10);
    Vertex* startingVertex = findVertex(startingID, myVector, position);
    if (startingVertex == nullptr) {
        return;
    }

    //Possible correction// while ((nd = nd->NextSiblingElement("nd")) != nullptr) {
    while (nd->NextSiblingElement("nd") != nullptr) {
        //Finding the Ending node.
        nd = nd->NextSiblingElement("nd");
        const char* refEndingID = nd->Attribute("ref");
        const unsigned long long endingID = strtoull(refEndingID, nullptr, 10);
        Vertex* endingVertex = findVertex(endingID, myVector, position);
        if (endingVertex == nullptr) {
            return;
        }

        Edge newEdge(startingVertex->getId(), endingID, (factor * haversineDistance(*startingVertex, *endingVertex)));
        startingVertex->addEdge(newEdge);

        if (!oneWay) {
            Edge reverseEdge(endingID, startingVertex->getId(), (factor * haversineDistance(*endingVertex, *startingVertex)));
            endingVertex->addEdge(reverseEdge);
        }

        startingVertex = endingVertex;
    }
}


//This method removes all the Vertexes from the vector that do not have edges and then updates the positions in the unordered map.
void Utilities::removeBlankNodes(std::vector<Vertex> &myVector, std::unordered_map<unsigned long long, unsigned int> &position) {

    std::vector<Vertex> filtered;
    std::unordered_map<unsigned long long, unsigned int> newPosition;

    filtered.reserve(myVector.size());
    newPosition.reserve(position.size());

    unordered_set<unsigned long long> endpoints;
    for (const Vertex &v : myVector) {
        for (const Edge &e : *v.getEdge()) {
            endpoints.insert(e.getEndId());
        }
    }

    for (const Vertex &v : myVector) {
        if (!v.getEdge()->empty() || endpoints.count(v.getId()) > 0) {
            newPosition[v.getId()] = filtered.size();
            filtered.push_back(v);
        }
    }

    myVector = std::move(filtered);
    position = std::move(newPosition);
}


void Utilities::createNodeEdge(XMLElement *root, vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position) {
    for (XMLElement* way = root->FirstChildElement("way"); way; way = way->NextSiblingElement("way")) {
        readRefs(way, myVector, position);
    }

    removeBlankNodes(myVector, position);
}


void Utilities::EdgeDeletion(Vertex &deletingVertex, vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position) {

    while (!deletingVertex.getEdge()->empty()) {
        unsigned long long int checkID = deletingVertex.getEdge()->begin()->getEndId();

        Vertex* checkVertex = findVertex(checkID, myVector, position);
        if (checkVertex != nullptr) {
            list<Edge> *edges = checkVertex->getEdge();
            for (Edge& edgeIt : *edges) {
                if (edgeIt.getEndId() == deletingVertex.getId()) {
                    checkVertex->removeEdge(edgeIt);
                }
            }
        }

        deletingVertex.removeEdge(deletingVertex.getEdge()->front());
    }
}


void Utilities::mapUpdates(const unsigned int numPos, const vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position) {
    for (unsigned int i = numPos; i < myVector.size(); i++) {
        unsigned long long updateID = myVector[i].getId();
        position.find(updateID)->second--;
    }
}


// void Utilities::PairsUpdate(set<pair<unsigned long long, double>, CompareBySecond> &toVisit, const pair<unsigned long long, double> &currentPair, Vertex &currentVertex) {
//
//     for (Edge& edge : *currentVertex.getEdge()) {
//         toVisit.emplace(edge.getEndId(), (edge.getDistance() + currentPair.second));
//     }
// }
//
//
// void Utilities::dijkstraRecursion(vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position, unordered_map<unsigned long long, tuple<unsigned long long, double>> &data, set<pair<unsigned long long,double>, CompareBySecond> &toVisit, vector<bool> &visited, pair<unsigned long long, double> &currentPair) {
//
//     //We find the vertex that corresponds to the current pair's ID
//     Vertex *currentVertex = findVertex(currentPair.first, myVector, position);
//     //We update the available moves
//     PairsUpdate(toVisit, currentPair, *currentVertex);
//
//     //Termination term
//     if (toVisit.empty()) {
//         return;
//     }
//
//     for (const auto &pairIterator : toVisit) {
//         //If the distance is smaller than the one we have, then we keep the smaller one along with the predecessor.
//         auto dataIterator = data.find(pairIterator.first);
//         if (get<1>(dataIterator->second) > pairIterator.second) {
//             get<0>(dataIterator->second) = currentPair.first;//predecessor
//             get<1>(dataIterator->second) = pairIterator.second;//new distance
//         }
//     }
//     //mark currentPair(predecessor) as visited
//     const auto it = position.find(currentPair.first);
//     visited[it->second] = true;
//
//     //Then we choose the next pair, if exists.
//     while (!toVisit.empty()) {
//         bool next;
//         do {
//             //We check if the next smallest pair is visited or not
//             set<pair<unsigned long long, double>, CompareBySecond>::iterator nextPair = toVisit.begin();
//             const auto check = position.find(nextPair->first);
//             next = visited[check->second];
//             //If it is
//             if (next == true) {
//                 //We check if it is a correct option or not
//                 if(nextPair->second != get<1>(data.find(nextPair->first)->second)) {
//                     //If not we erase it
//                     toVisit.erase(nextPair);
//                 }
//                 else {
//                     // we accept
//                     next = false;
//                     currentPair = *nextPair;
//                 }
//             }
//             else {
//                 currentPair = *nextPair;
//             }
//
//             if (toVisit.empty()) {
//                 return;
//             }
//         }while (next == true);
//
//         //We repeat
//         dijkstraRecursion(myVector, position, data, toVisit, visited, currentPair);
//         //At the end we remove the used pair
//         toVisit.erase(toVisit.begin());
//     }
// }


void Utilities::dijkstraIterative(vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position, unordered_map<unsigned long long, tuple<unsigned long long, double>> &data, const pair<unsigned long long, double> &startPair) {

    //We create a set of pairs with the nodes that we have direct access to, in ascending order to the distance
    set<pair<unsigned long long, double>, CompareBySecond> toVisit;
    vector<bool> visited(myVector.size(), false);

    // Initialize toVisit
    toVisit.insert(startPair);
    get<1>(data[startPair.first]) = startPair.second; // distance = 0
    get<0>(data[startPair.first]) = startPair.first;  // predecessor = self

    while (!toVisit.empty()) {
        // We choose the pair with the smallest distance from the current pair
        auto it = toVisit.begin();
        pair<unsigned long long, double> currentPair = *it;
        toVisit.erase(it);

        const auto posIt = position.find(currentPair.first);
        if (visited[posIt->second]){
            continue; // If we have already visited this pair, skip
        }
        visited[posIt->second] = true;

        // Current vertex handling
        Vertex *currentVertex = findVertex(currentPair.first, myVector, position);
        if (!currentVertex) {
            continue;
        }

        // neighbors update
        for (Edge &edge : *currentVertex->getEdge()) {
            unsigned long long neighborId = edge.getEndId();
            double newDist = currentPair.second + edge.getDistance();

            auto &neighborData = data[neighborId];
            if (newDist < get<1>(neighborData)) {
                get<0>(neighborData) = currentPair.first;  // predecessor
                get<1>(neighborData) = newDist;           // new distance
                toVisit.insert({neighborId, newDist});    // add in set
            }
        }
    }
}


void Utilities::merging(vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position, const int mode, const Edge &start, Vertex &mid, const Edge &end) {

    if (mode == 1) {
        Vertex *startVertex = findVertex(start.getStartId(), myVector, position);
        Vertex *endVertex = findVertex(end.getEndId(), myVector, position);

        double factor = start.getDistance() / haversineDistance(*startVertex, mid);
        Edge newEdge(start.getStartId(), endVertex->getId(), factor * haversineDistance(*startVertex, *endVertex));

        startVertex->removeEdge(start);
        startVertex->addEdge(newEdge);
        mid.removeEdge(end);
    }

    else if (mode == 2) {
        Vertex *startVertex = findVertex(start.getStartId(), myVector, position);
        Vertex *endVertex = findVertex(end.getEndId(), myVector, position);

        double factor1 = start.getDistance() / haversineDistance(*startVertex, mid);
        double factor2 = end.getDistance() / haversineDistance(*endVertex, mid);

        Edge newEdge1(start.getStartId(), end.getEndId(), factor1 * haversineDistance(*startVertex, *endVertex));
        Edge newEdge2(end.getEndId(), start.getStartId(), factor2 * haversineDistance(*endVertex, *startVertex));

        list<Edge> *edgeList = mid.getEdge();
        edgeList->clear();

        edgeList = startVertex->getEdge();
        for (Edge &edge : *edgeList) {
            if (edge.getEndId() == mid.getId()) {
                startVertex->removeEdge(edge);
                break;
            }
        }

        edgeList = endVertex->getEdge();
        for (Edge &edge : *edgeList) {
           if (edge.getEndId() == mid.getId()) {
               endVertex->removeEdge(edge);
               break;
           }
        }

        startVertex->addEdge(newEdge1);
        endVertex->addEdge(newEdge2);
    }


}


void Utilities::bfsStructure(vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position, queue<Vertex*> &FIFO, Vertex *startingPoint) {
    vector<bool> visited(myVector.size(), false);
    queue<Vertex*> toVisit;

    toVisit.push(startingPoint);
    auto startIt = position.find(startingPoint->getId());
    if (startIt == position.end()) {
        return;
    }
    visited[startIt->second] = true;

    while (!toVisit.empty()) {
        Vertex *currentVertex = toVisit.front();
        toVisit.pop();

        FIFO.push(currentVertex);

        const list<Edge> *edgeList = currentVertex->getEdge();

        vector<Vertex*> neighbors;
        for (const Edge &edge : *edgeList) {
            Vertex* neighbor = findVertex(edge.getEndId(), myVector, position);
            if (neighbor) {
                auto it = position.find(neighbor->getId());
                if (!visited[it->second]) {
                    neighbors.push_back(neighbor);
                }
            }
        }

        sort(neighbors.begin(), neighbors.end(), [](const Vertex* a, const Vertex* b) { return a->getId() < b->getId(); });

        for (Vertex* neighbor : neighbors) {
            auto it = position.find(neighbor->getId());
            visited[it->second] = true;
            toVisit.push(neighbor);
        }
    }
}


void Utilities::dfsStructure(vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position, Vertex *startingPoint, list<unsigned long long> &result) {
    vector<bool> visited(myVector.size(), false);
    stack<Vertex*> toVisit;

    toVisit.push(startingPoint);

    while (!toVisit.empty()) {
        Vertex *currentVertex = toVisit.top();
        toVisit.pop();

        auto currIt = position.find(currentVertex->getId());
        if (visited[currIt->second]) {
            continue;
        }
        visited[currIt->second] = true;

        result.push_back(currentVertex->getId());

        const list<Edge> *edgeList = currentVertex->getEdge();

        vector<Vertex*> neighbors;
        for (const Edge &edge : *edgeList) {
            Vertex* neighbor = findVertex(edge.getEndId(), myVector, position);
            if (neighbor) {
                auto it = position.find(neighbor->getId());
                if (!visited[it->second]) {
                    neighbors.push_back(neighbor);
                }
            }
        }

        sort(neighbors.begin(), neighbors.end(), [](const Vertex* a, const Vertex* b) {return a->getId() > b->getId(); });

        for (Vertex* neighbor : neighbors) {
            toVisit.push(neighbor);
        }
    }
}


// void Utilities::printPath(vector<Vertex> &myVector, unordered_map<unsigned long long, unsigned int> &position, list<Edge> &path) {
//
//     double totalDistance = 0.0;
//
//     for (Edge &edge : path) {
//         totalDistance += edge.getDistance();
//
//         cout << "[" << edge.getStartId() << " -> " << edge.getEndId() << "] ";
//         cout << fixed << setprecision(3) << totalDistance << endl;
//     }
//
//     cout << "\n" << "\n" << "https://www.google.com/maps/dir/";
//
//     for (Edge &edge : path) {
//         Vertex* vertexIt = findVertex(edge.getEndId(), myVector, position);
//
//         cout << vertexIt->getLatitude() << "," << vertexIt->getLongitude() << "/";
//     }
//
//     cout << endl;
// }

void Utilities::printPath(vector<Vertex> &myVector,
                          unordered_map<unsigned long long, unsigned int> &position,
                          list<Edge> &path) {
    double totalDistance = 0.0;

    for (Edge &edge : path) {
        totalDistance += edge.getDistance();
        cout << "[" << edge.getStartId() << " -> " << edge.getEndId() << "] "
             << fixed << setprecision(3) << totalDistance << '\n';
    }

    cout << '\n' << '\n' << "https://www.google.com/maps/dir/";

    if (!path.empty()) {
        auto first = path.front();
        if (auto *vStart = findVertex(first.getStartId(), myVector, position)) {
            cout << vStart->getLatitude() << "," << vStart->getLongitude() << "/";
        }
        for (Edge &edge : path) {
            if (auto *vEnd = findVertex(edge.getEndId(), myVector, position)) {
                cout << vEnd->getLatitude() << "," << vEnd->getLongitude() << "/";
            }
        }
    }
    cout << '\n';
}


void Utilities::printFS(const std::list<unsigned long long> &result) {
    for (const unsigned long long &i : result) {
        cout << i << "\n";
    }
}