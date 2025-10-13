//
// Created by nkara on 11/6/2025.
//

#ifndef EDGE_H
#define EDGE_H



class Edge {
    private:
        unsigned long long startId;
        unsigned long long endId;
        double distance;

    public:
        Edge() = default;
        Edge(unsigned long long startId, unsigned long long endId, double distance);
        Edge(const Edge &copyEdge);

        [[nodiscard]] unsigned long long getStartId() const;
        [[nodiscard]] unsigned long long getEndId() const;
        [[nodiscard]] double getDistance() const;

        void setStartId(unsigned long long start_id) ;

        void setEndId(unsigned long long end_id);

        void setDistance(double dist);

        bool operator == (const Edge &edge) const;
};



#endif //EDGE_H
