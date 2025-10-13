//
// Created by nkara on 11/6/2025.
//

#include "Edge.hpp"

Edge::Edge(const unsigned long long startId, const unsigned long long endId, const double distance) {
    this->startId = startId;
    this->endId = endId;
    this->distance = distance;
}

Edge::Edge(const Edge &copyEdge) {
    this->startId = copyEdge.startId;
    this->endId = copyEdge.endId;
    this->distance = copyEdge.distance;
}

unsigned long long Edge::getStartId() const {
    return startId;
}

unsigned long long Edge::getEndId() const {
    return this->endId;
}

double Edge::getDistance() const {
    return  this->distance;
}

void Edge::setStartId(unsigned long long start_id) {
    this->startId = start_id;
}

void Edge::setEndId(unsigned long long end_id) {
    this->endId = end_id;
}

void Edge::setDistance(double dist) {
    this->distance = dist;
}

bool Edge::operator == (const Edge &edge) const {
    if (this->startId == edge.startId && this->endId == edge.endId) {
        return this->distance == edge.distance;
    }
    return false;
}
