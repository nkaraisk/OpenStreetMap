//
// Created by nkara on 11/6/2025.
//

#include "Vertex.hpp"

using namespace std;

//Constructors
Vertex::Vertex(const unsigned long long id, const double latitude, const double longitude) {
    this->id = id;
    this->latitude = latitude;
    this->longitude = longitude;
}

Vertex::Vertex(const Vertex &copyVertex) {
    this->id = copyVertex.id;
    this->latitude = copyVertex.latitude;
    this->longitude = copyVertex.longitude;

    for (Edge const &edge : copyVertex.edgeList) {
        this->edgeList.emplace_back(edge);
    }
}


//Methods
void Vertex::addEdge(const Edge &edge) {
    this->edgeList.push_back(edge);
}

void Vertex::removeEdge(const Edge &edge) {
    this->edgeList.remove(edge);
}

//Getters
unsigned long long Vertex::getId() const {
    return this->id;
}

double Vertex::getLatitude() const {
    return this->latitude;
}

double Vertex::getLongitude() const {
    return this->longitude;
}

std::list<Edge>* Vertex::getEdge() {
    return &this->edgeList;
}
