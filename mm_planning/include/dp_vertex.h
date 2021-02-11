//
// Created by hilsys on 2021/2/11.
//

#ifndef MM_PLANNING_DP_VERTEX_H
#define MM_PLANNING_DP_VERTEX_H



#include "vec2d.h"
#include <limits>
#include <cmath>



class Vertex {
private:
    Vec2d *current_vertex_;
    Vertex *parent_vertex_;
    double cost_from_start_;
public:

    Vertex(Vec2d *current_vertex, Vertex *parent_vertex, double cost) :
            current_vertex_(current_vertex),
            parent_vertex_(parent_vertex),
            cost_from_start_(cost) {}

    Vertex() : Vertex(nullptr, nullptr, std::numeric_limits<double>::max()){}

    void set_parent(Vertex* parent){parent_vertex_=parent;}

    void set_cost(double cost){cost_from_start_=cost;
    }

    Vec2d get_position(){return *current_vertex_;}

    Vec2d* get_position_ptr(){return current_vertex_;}

    Vertex* get_parent(){return parent_vertex_;}

    double get_cost(){return cost_from_start_;}

    double distance_to(Vertex another){
        return hypot(current_vertex_->x()-another.get_position().x(),current_vertex_->y()-another.get_position().y());
    }


};

#endif //MM_PLANNING_DP_VERTEX_H
