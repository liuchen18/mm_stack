//
// Created by hilsys on 2021/1/20.
//

#ifndef MM_PLANNING_MM_PLANNER_H
#define MM_PLANNING_MM_PLANNER_H

#include "point3d.h"
#include "dynamic_voronoi.h"
#include "pose2d.h"
#include "vec2d.h"
#include <cmath>
#include "dp_vertex.h"

class mm_planner{
private:
    int max_iteration_;
    DynamicVoronoi voronoi_; ///voronoi diagram of the map
    cv::Mat map_img_; /// the input map
    int map_width_;
    int map_height_;
    std::vector<Point3d> ee_path_; ///desired ee path
    std::vector<Point3d> parse_ee_path_;
    std::vector<Pose2d> base_origin_path_; ///original base path
    std::vector<Pose2d> parse_base_origin_path_;
    std::vector<Pose2d> optimized_base_path_; ///planned base path

    double weight_smoothness_=0.1;
    double weight_bound_=10.0;
    double weight_distance_=0.1;
    double weight_voronoi_=0.1;

    int map_resolution_=10;

    ///the threshold of the voronoi term, if the distance between the closed obstacle and current position is larger
    ///than this value, voronoi term will not work
    double vorObsDMax_=30;

    /// falloff rate for the voronoi field
    float alpha_ = 1;

    double base_height_=400;

    bool is_dp_;

    int inter_num_=10;


public:
    /**
     * constructor
     * @param img the input map
     * @param ee_path the desired end effector path
     */
    mm_planner(cv::Mat img,const std::vector<Point3d>& ee_path,bool is_dp);

    /**
     * default constructor
     */
    mm_planner();

    /**
     * set the ee path
     * @param ee_path the given ee path
     */
    void set_ee_path(std::vector<Point3d> ee_path);

    /**
     * get the desired ee path
     * @return
     */
    std::vector<Point3d> get_ee_path(){return ee_path_;}

    /**
     * get the planned path of the base
     * @return
     */
    std::vector<Pose2d> get_base_path(){return optimized_base_path_;}

    /**
     * plan the base path
     */
    void plan();

    /**
     * compute the initial base path, the origin base path is updated and the parse path is returned
     * @param ee_path
     * @return
     */
    std::vector<Pose2d> compute_origin_base_path(std::vector<Point3d> ee_path,bool is_dp);

    /**
     * get the sampled point according to the given end effector point
     * @param ee_point Point3d, the given end effector point
     * @param sample_number sanmpled point number
     * @return the vector of all the sampled points
     */
    std::vector<Vec2d*> get_sample_points(Point3d ee_point,int sample_number);

    /**
     * compute a original base path according to the given sampled points
     * @param sampled_points
     * @return original base path
     */
    std::vector<Vec2d*> dynamic_programming(std::vector<std::vector<Vec2d*>> sampled_points);

    /**
     * delete the useless sampled points
     * @param sampled_points
     * @param dp_path
     */
    void clear_useless_points(std::vector<std::vector<Vec2d*>> sampled_points,std::vector<Vec2d*> dp_path);

    /**
     * convert the vector of pointers to a vector of pose2d and delete the pointers
     * @param dp_path
     * @param path
     */
    void convert_ptr_vec(std::vector<Vec2d*>& dp_path,std::vector<Pose2d>& path);

    /**
     * compute the inner bound of the given height
     * @param height the desired ee height,mm
     * @return inner bound
     */
    double inner_bound(double height);

    /**
     * compute the outer bound of the given height
     * @param height the desired ee height,mm
     * @return outer bound
     */
    double outer_bound(double height);

    /**
     * set the max iteration for the gradient decent
     * @param iteration
     */
    void set_max_iteration(int iteration){max_iteration_=iteration;}

    /**
     * set the weight of the bound term
     * @param weight_bound
     */
    void set_bound_weight(double weight_bound){weight_bound_=weight_bound;}

    /**
     * set the weight of the smoothness term
     * @param smoothness_weight
     */
    void set_smoothness_weight(double smoothness_weight ){weight_smoothness_=smoothness_weight;}

    /**
     * set the weight of the distance term
     * @param distance_weight
     */
    void set_distance_weight(double distance_weight){weight_distance_=distance_weight;}

    /**
     *
     * @return weight bound term
     */
    double get_weight_bound(){return weight_bound_;}

    /**
     *
     * @return weight smoothness term
     */
    double get_weight_smoothness(){return weight_smoothness_;}

    /**
     *
     * @return weight distance term
     */
    double get_weight_distance(){return weight_distance_;}

    /**
     * compute the distance term of the given point, the point is given by its index
     * @param pt_index the index of the given point
     * @return the distance term of the given point
     */
    Vec2d distance_term(int pt_index);

    /**
     * compute the bound term of the given point, the point is given by its index
     * @param pt_index the index of the given point
     * @return the bound term of the given point
     */
    Vec2d bound_term(int pt_index);

    /**
     * compute the smoothness term of the given point, the point is given by its index
     * @param pt_index the index of the given point
     * @return the smoothness term of the given point
     */
    Vec2d smoothness_term(int pt_index);

    /**
     * compute the voronoi term of the given point,the point is given by its index
     * @param pt_index
     * @return the voronoi term of the given point
     */
    Vec2d voronoi_term(int pt_index);

    /**
     * check whether the position in the given boundary
     * @param ee_pt the ee position
     * @param base_pos the given base position
     * @return true if in
     */
    bool is_in_boundary(Point3d ee_pt,Vec2d base_pos);

    /**
     *
     * @param data
     */
    void print_vector_pose2d(std::vector<Pose2d> data);

    /**
     *
     */
    void show_final_path();

    /**
     * check if the path between the two vertex is collision free
     * @param v1
     * @param v2
     * @return true if no collision free
     */
    bool is_collision_free(Vertex v1,Vertex v2);

    /**
     *
     * @param point
     * @return
     */
    bool is_collision_free(Vec2d point);

    bool is_collision_free(Vec2d* v1,Vec2d* v2);

    bool is_collision_free(Vec2d v1,Vec2d v2);

    /**
     *
     * @param origin_path
     * @param inter_num
     * @return
     */
    std::vector<Pose2d> parse_path(std::vector<Pose2d> origin_path,int inter_num);

    /**
     *
     * @param origin_path
     * @param inter_num
     * @return
     */
    std::vector<Point3d> parse_path(std::vector<Point3d> origin_path,int inter_num);

    void show_sampled_points(std::vector<std::vector<Vec2d*>> sampled_points);

    void show_collision_edge(Vertex p1,Vertex p2);

    void show_collision_point(Vec2d p,int size);

};


#endif //MM_PLANNING_MM_PLANNER_H
