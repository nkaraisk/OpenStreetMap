//
// Created by nkara on 11/6/2025.
//

#ifndef VERTEX_H
#define VERTEX_H

#include "Edge.hpp"

#include <list>


class Vertex {
    private:
        unsigned long long id;
        double latitude;
        double longitude;
        std::list<Edge> edgeList;

    public:
        Vertex() = default;
        Vertex(unsigned long long id, double latitude, double longitude);
        Vertex(const Vertex &copyVertex);

        void addEdge(const Edge& edge);
        void removeEdge(const Edge& edge);

        [[nodiscard]] unsigned long long getId() const;
        [[nodiscard]] double getLatitude() const;
        [[nodiscard]] double getLongitude() const;
        [[nodiscard]] std::list<Edge>* getEdge();
        [[nodiscard]] const std::list<Edge>* getEdge() const {
            return &edgeList;
        }

        struct CompareVertex {
            bool operator()(const Vertex& a, const Vertex& b) const {
                return a.getId() > b.getId();
            }
        };

};

#endif //VERTEX_H