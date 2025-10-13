//
// Created by nkara on 11/6/2025.
//

#include "Graph.hpp"


#include <climits>
#include <cfloat>
#include <iostream>
#include <utility>

using namespace std;
using namespace tinyxml2;

//Constructors
Graph::Graph() = default;


Graph::Graph(const string &filename) {
    vector<Vertex> myVector;
    unordered_map<unsigned long long, unsigned int> myPosition;

    XMLDocument doc = XMLDocument();
    XMLError result = doc.LoadFile(filename.c_str());

    if ((result != XML_SUCCESS) && (result!= XML_ERROR_PARSING_TEXT)){
        cout << "\n" << "Unable to open file: " << filename << endl;
    }
    else if (result == XML_ERROR_PARSING_TEXT) {
        cout << "\n" << "Invalid format for file: " << filename << endl;
    }
    else {
        XMLElement* root = doc.FirstChildElement("osm");
        if (!root) {
            cout << "\n" << "Invalid format for file: " << filename << endl;
        }
        else {
            Utilities::createVertexList(root, myVector, myPosition);
            Utilities::createNodeEdge(root, myVector, myPosition);
        }
    }

    this->vertexMap = std::move(myVector);
    this->position = std::move(myPosition);
}


bool Graph::addVertex(unsigned long long id, double lat, double lon) {
    Vertex *result = Utilities::findVertex(id, this->vertexMap, this->position);
    if (result == nullptr) {
        vertexMap.emplace_back(id, lat, lon);
        position.emplace(id, vertexMap.size() - 1);
        return true;
    }

    return false;
}


bool Graph::deleteVertex(const unsigned long long id) {
    Vertex *deletingVertex = Utilities::findVertex(id, this->vertexMap, this->position);

    if (deletingVertex == nullptr) {
        cerr << "No <osm> element found!" << endl;
        return false;
    }

    Utilities::EdgeDeletion(*deletingVertex, this->vertexMap, this->position);

    const auto positionIndex = position.find(deletingVertex->getId());
    const unsigned int numPos = positionIndex->second;

    position.erase(positionIndex);
    vertexMap.erase(vertexMap.begin()+numPos);

    Utilities::mapUpdates(numPos, this->vertexMap, this->position);

    return true;
}


list<Edge> Graph::shortestPath(const unsigned long long source, unsigned long long destination) {

    unordered_map<unsigned long long, tuple<unsigned long long, double>> data;

    for (Vertex& vertexIt : vertexMap) {
        if (vertexIt.getId() == source) {
            data.emplace(source, make_tuple(source, 0));
        }
        else {
            data.emplace(vertexIt.getId(), make_tuple(ULLONG_MAX, DBL_MAX));
        }
    }

    const pair<unsigned long long,double> startPair = make_pair(source, 0);
    Utilities::dijkstraIterative(vertexMap, position, data, startPair);

    list<Edge> path;
    auto endPointIt = data.find(destination);

    if (endPointIt == data.end() || get<1>(endPointIt->second) == DBL_MAX) {
        cerr << "No path exists from " << source << " to " << destination << endl;
        return path; // empty path
    }

    while (true) {
        Vertex *vertexIt = Utilities::findVertex(get<0>(endPointIt->second), this->vertexMap, this->position);
        if (!vertexIt) {
            cerr << "No element found for id " << endPointIt->first << endl;
            break;
        }

        Edge *selectedEdge = nullptr;
        for (Edge &edgeIt : *vertexIt->getEdge()) {
            if (edgeIt.getEndId() == endPointIt->first) {
                selectedEdge = &edgeIt;
                break;
            }
        }

        if (!selectedEdge) {
            cerr << "Edge not found from " << vertexIt->getId() << " to " << endPointIt->first << endl;
            break;
        }

        path.emplace_front(*selectedEdge);

        if (vertexIt->getId() == source) {
            break;
        }

        unsigned long long predecessor = get<0>(endPointIt->second);
        if (predecessor == ULLONG_MAX) {
            break;
        }

        endPointIt = data.find(predecessor);
        if (endPointIt == data.end()) {
            break;
        }
    }

    return path;
}


void Graph::graphCompression() {

    bool change;
    do {
        change = false;

        for (Vertex& vertexIt : vertexMap) {
            list<Edge> *edgeList = vertexIt.getEdge();

            int startNodes = 0;
            list<Edge> startingNodes;

            if (edgeList->size() == 1) {

                for (Vertex &startingVertex : vertexMap) {
                    if (startingVertex.getId() == vertexIt.getId()) {
                        continue;
                    }

                    list<Edge> *checkList = startingVertex.getEdge();
                    for (Edge &edgeIt : *checkList) {
                        if (edgeIt.getEndId() == vertexIt.getId()) {
                            startingNodes.push_back(edgeIt);
                            startNodes++;
                        }
                    }
                }

                if (startNodes == 1) {
                    //Merge
                    Utilities::merging(vertexMap, position, 1, startingNodes.front(), vertexIt, edgeList->front());
                    change = true;
                    break;
                }

                startingNodes.clear();
            }
            else if (edgeList->size() == 2) {

                for (Vertex &startingVertex : vertexMap) {
                    if (startingVertex.getId() == vertexIt.getId()) {
                        continue;
                    }

                    list<Edge> *checkList = startingVertex.getEdge();
                    for (Edge &edgeIt : *checkList) {
                        if (edgeIt.getEndId() == vertexIt.getId()) {
                            startingNodes.push_back(edgeIt);
                            startNodes++;
                        }
                    }
                }

                int check = 0;
                for (Edge &edgeIt : *edgeList) {
                    for (Edge &checkIt : startingNodes) {
                        if ((edgeIt.getEndId() == checkIt.getEndId()) && (edgeIt.getStartId() == checkIt.getStartId())) {
                            check++;
                        }
                    }
                }

                if ((startNodes == 2) && (check == 2)) {
                    if (edgeList->front().getEndId() == startingNodes.front().getStartId()) {
                        //Merge
                        Utilities::merging(vertexMap, position, 2,startingNodes.back(), vertexIt, edgeList->front());
                    }
                    else {
                        //Merge
                        Utilities::merging(vertexMap, position, 2,startingNodes.front(), vertexIt, edgeList->front());
                    }
                    change = true;
                    break;
                }

                startingNodes.clear();
            }
        }

        Utilities::removeBlankNodes(vertexMap, position);
    }while (change);
}


list<unsigned long long> Graph::BFS(unsigned long long start) {
    list<unsigned long long> result;
    queue<Vertex*> FIFO;

    Vertex *startingPoint = Utilities::findVertex(start, vertexMap,position);

    Utilities::bfsStructure(vertexMap, position, FIFO, startingPoint);

    while (!FIFO.empty()) {
        result.emplace_back(FIFO.front()->getId());
        FIFO.pop();
    }

    return result;
}

list<unsigned long long> Graph::DFS(unsigned long long start) {
    list<unsigned long long> result;

    Vertex *startingPoint = Utilities::findVertex(start, vertexMap,position);

    Utilities::dfsStructure(vertexMap, position, startingPoint, result);

    return result;
}

