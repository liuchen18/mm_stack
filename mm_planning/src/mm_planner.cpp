//
// Created by hilsys on 2021/1/20.
//
#include "mm_planner.h"
#include <algorithm>
#define pi 3.1415926

mm_planner::mm_planner(cv::Mat map_img,const std::vector<Point3d>& ee_path,bool is_dp) {
    map_height_ = map_img.rows;
    map_width_ = map_img.cols;
    if(map_img.channels() != 1) {
        map_img_ = map_img.clone();
        cv::cvtColor(map_img, map_img_, CV_8UC1);
    } else {
        cv::cvtColor(map_img, map_img_, CV_8UC1);
    }
    voronoi_.buildVoronoiFromImage(map_img);

    if(map_img_.channels() != 1) {
        cv::cvtColor(map_img_, map_img_, cv::COLOR_BGR2GRAY);
    }

    for(const auto& pt : ee_path) {
        ee_path_.emplace_back(pt.x(), pt.y(), pt.z());
    }
    is_dp_=is_dp;
    //imshow("map", map_img_);
    //cv::waitKey();
}

void mm_planner::set_ee_path(std::vector<Point3d> ee_path) {
    for(const auto& pt : ee_path) {
        ee_path_.emplace_back(pt.x(), pt.y(), pt.z());
    }
}

void mm_planner::plan() {
    std::cout<<"the max iteration number is "<<max_iteration_<<std::endl;
    int iterations = 0;
    compute_origin_base_path(ee_path_,is_dp_);
    std::cout<<"original base path is found! "<<std::endl;
    //print_vector_pose2d(base_origin_path_);

    base_path_=base_origin_path_;

    double totalWeight = weight_bound_ + weight_distance_ + weight_smoothness_;

    while(iterations < max_iteration_){
        iterations+=1;
        for(int i=1;i<base_path_.size()-1;i++){
            Vec2d current(base_path_[i].x(),base_path_[i].y());
            Vec2d correction;
            //std::cout<<"base position"<<current.x()<<" "<<current.y()<<std::endl;

            correction=correction-distance_term(i)*weight_distance_;

            if(!is_in_boundary(ee_path_[i],current+correction)){
                continue;
            }
            correction=correction-bound_term(i)*weight_bound_;
            if(!is_in_boundary(ee_path_[i],current+correction)){
                continue;
            }
            correction=correction-smoothness_term(i)*weight_smoothness_;
            if(!is_in_boundary(ee_path_[i],current+correction)){
                continue;
            }
            //std::cout<<"base position"<<current.x()<<" "<<current.y()<<std::endl;
            //std::cout<<"ee position"<<ee_path_[i].x()/map_resolution_<<" "<<ee_path_[i].y()/map_resolution_<<std::endl;
            //std::cout<<"smoothness: "<<smoothness_term(i).x()<<" "<<smoothness_term(i).y()<<std::endl;
            //std::cout << "bound: " << 10 * bound_term(i).x() << " " << 10 * bound_term(i).y() << std::endl;
            //std::cout<<"correction: "<<correction.x()<<" "<<correction.y()<<std::endl;
            correction=correction-voronoi_term(i)*weight_voronoi_;
            if(!is_in_boundary(ee_path_[i],current+correction)){
                continue;
            }
            if(!is_collision_free(Vec2d(base_path_[i].x(),base_path_[i].y()),current+correction)){
                continue;
            }
            current = current +correction;
            base_path_[i].set_x(current.x());
            base_path_[i].set_y(current.y());
        }
        //show_final_path();
    }

    show_final_path();
}

std::vector<Pose2d> mm_planner::compute_origin_base_path(std::vector<Point3d> ee_path,bool is_dp) {
    std::vector<Pose2d> origin_base_path;
    if(!is_dp) {
        std::cout<<"start planning the original base path using the projection method!"<<std::endl;
        for (auto &pt:ee_path) {
            double i_bound = inner_bound(pt.z() - base_height_);
            double o_bound = outer_bound(pt.z() - base_height_);
            //here the position is divided by map_resolution
            origin_base_path.emplace_back((pt.x() - (i_bound + o_bound) / 2) / map_resolution_,
                                          (pt.y()) / map_resolution_, 0);
        }
    }
    else{
        std::cout<<"start planning the original base path using the dynamic programming method!"<<std::endl;
        std::vector<std::vector<Vec2d*>> sampled_points;
        for(auto &ee_point: ee_path){
            std::vector<Vec2d*> current_sampled_points=get_sample_points(ee_point,10);
            sampled_points.push_back(current_sampled_points);
        }
        //std::cout<<"get all the sampled points"<<std::endl;

        std::vector<Vec2d*> dp_path=dynamic_programming(sampled_points);
        //std::cout<<"dynamic programming finished!"<<std::endl;

        clear_useless_points(sampled_points,dp_path);

        convert_ptr_vec(dp_path,origin_base_path);

    }
    base_origin_path_=origin_base_path;
    return origin_base_path;
}

void mm_planner::convert_ptr_vec(std::vector<Vec2d *> &dp_path, std::vector<Pose2d> &path) {
    for(int i=0;i<dp_path.size();i++){
        path.emplace_back(dp_path[i]->x(),dp_path[i]->y(),0);
    }
    for(auto& point:dp_path){
        delete point;
    }

}

std::vector<Vec2d*> mm_planner::dynamic_programming(std::vector<std::vector<Vec2d *>> sampled_points) {
    std::vector<Vec2d*> dp_path;
    std::vector<std::vector<Vertex>> graph;

    //construct the graph.
    for(auto cols:sampled_points){
        std::vector<Vertex> col_graph;
        for(auto point:cols){
            Vertex cur_vertex(point, nullptr,std::numeric_limits<double>::max());
            col_graph.push_back(cur_vertex);
        }
        graph.push_back(col_graph);
    }
    //std::cout<<"graph contructed!"<<std::endl;

    //the cost of the first colum is 0
    for(auto &vertex:graph[0]){
        vertex.set_cost(0);
        //std::cout<<vertex.get_cost()<<" "<<graph[0][1].get_cost()<<std::endl;
    }
    //std::cout<<graph[0][0].get_cost()<<std::endl;

    //for every colum
    for(int i=1;i<graph.size();i++){
        //for every point in next colum
        for(auto &vertex:graph[i]){
            if(!is_collision_free(vertex.get_position())){
                //std::cout<<"point collision"<<std::endl;
                continue;
            }
            //for every possible parent
            for(auto &possible_parent:graph[i-1]){
                //if collision free, compute the distance
                if(is_collision_free(possible_parent,vertex) && is_collision_free(vertex,possible_parent)){
                    double current_cost=vertex.distance_to(possible_parent);
                    //if the vertex has no parent or the new path cost is smaller
                    if(!vertex.get_parent() || current_cost+possible_parent.get_cost()<vertex.get_cost()){
                        vertex.set_parent(&possible_parent);
                        vertex.set_cost(possible_parent.get_cost()+current_cost);
                    }
                }
                else{
                    //std::cout<<"collision"<<std::endl;
                }
            }
        }
    }
    //std::cout<<"dp finished"<<std::endl;

    //get the last vertex in the last colum
    Vertex final_vertex;
    for(auto &vertex:graph.back()){
        if(vertex.get_cost()<final_vertex.get_cost()){
            final_vertex=vertex;
        }
    }
    std::cout<<"final vertex found, and the length is : "<<final_vertex.get_cost() <<std::endl;

    //get the path with the min distance
    for(int i=0;i<graph.size();i++){
        dp_path.push_back(final_vertex.get_position_ptr());
        if(final_vertex.get_parent()) {
            final_vertex = *(final_vertex.get_parent());
        }
        else{
            if(i!=graph.size()-1){
                std::cerr<<"invalid dp path"<<std::endl;
            }
        }
    }
    for(int i=0;i<dp_path.size()-1;i++){
        bool c=is_collision_free(dp_path[i],dp_path[i+1]);
        if(!c){
            std::cout<<"collision!"<<std::endl;
        }
    }

    std::reverse(dp_path.begin(),dp_path.end());

    return dp_path;

}

bool mm_planner::is_collision_free(Vertex v1, Vertex v2) {
    double distance=v1.distance_to(v2);
    double angle=atan2(v2.get_position().y()-v1.get_position().y(),v2.get_position().x()-v1.get_position().y());
    int i=0;
    while(i<distance){
        i+=1;
        Vec2d cur(v1.get_position().x()+i*cos(angle),v1.get_position().y()+i*sin(angle));
        if(map_img_.at<uchar>(int(cur.y()),int(cur.x()))!=255){
            //std::cout<<"collision"<<std::endl;
            return false;
        }
    }

    return true;
}

bool mm_planner::is_collision_free(Vec2d *v1, Vec2d *v2) {
    Vertex v11(v1, nullptr,0),v22(v2, nullptr,0);
    return is_collision_free(v11,v22);
}

bool mm_planner::is_collision_free(Vec2d v1, Vec2d v2) {
    Vertex v11(&v1, nullptr,0),v22(&v2, nullptr,0);
    return is_collision_free(v11,v22);
}

bool mm_planner::is_collision_free(Vec2d point){
    if(map_img_.at<uchar>(int(point.y()),int(point.x()))!=255){
        //std::cout<<"collision"<<std::endl;
        return false;
    }
    return true;
}

void mm_planner::clear_useless_points(std::vector<std::vector<Vec2d *>> sampled_points, std::vector<Vec2d *> dp_path) {
    for(int i=0;i<dp_path.size();i++){
        for(auto &vec:sampled_points[i]){
            if(dp_path[i]!=vec){
                delete vec;
            }
            else{
                continue;
            }
        }
    }

}

std::vector<Vec2d*> mm_planner::get_sample_points(Point3d ee_point, int sample_number) {
    std::vector<Vec2d*> sampled_points;

    double i_bound=inner_bound(ee_point.z()-base_height_);
    double o_bound=outer_bound(ee_point.z()-base_height_);
    double redius=(i_bound+o_bound)/2;
    for(int i=0;i<sample_number;i++){
        double angle=i*1.0/sample_number*pi;
        double x=ee_point.x()-sin(angle)*redius;
        double y=ee_point.y()-cos(angle)*redius;
        Vec2d* sampled_point=new Vec2d(x/map_resolution_,y/map_resolution_);
        sampled_points.push_back(sampled_point);

    }
    return sampled_points;

}

double mm_planner::inner_bound(double height) {
    //            4            3            2
    //-1.004e-08 z + 1.42e-05 z - 0.008378 z + 2.323 z + 115.9
    double bound;
    if(height < 750 && height > 1) {
        bound = -1.004e-08 * pow(height, 4) + 1.42e-05 * pow(height, 3) - 0.008378 * pow(height, 2) + 2.323 * height +115.9;
    }
    else if(height>= 750 && height < 1300){
        bound=0;
    }
    else{
        std::cout<<"invalid height for the bound"<<std::endl;
        exit(0);
    }
    return bound;
}

double mm_planner::outer_bound(double height) {
    //           4             3            2
    //-3.14e-09 x + 6.064e-06 x - 0.004525 x + 1.365 x + 725
    double bound;
    if(height<1120 && height>1){
        bound=-3.14e-09*pow(height,4)+6.064e-06*pow(height,3)-0.004525*pow(height,2)+1.365*height+725;
    }
    else{
        std::cout<<"invalid height for the bound"<<std::endl;
        exit(0);
    }
    return bound;
}

Vec2d mm_planner::distance_term(int pt_index) {
    Vec2d dis_term;
    if(pt_index>0 && pt_index<base_path_.size()-1) {
        Vec2d current(base_path_[pt_index].x(),base_path_[pt_index].y());
        Vec2d next(base_path_[pt_index+1].x(),base_path_[pt_index+1].y());
        Vec2d last(base_path_[pt_index-1].x(),base_path_[pt_index-1].y());
        dis_term=4*current-2*last-2*next;
        return dis_term;
    }
    else{
        return dis_term;
    }

}

Vec2d mm_planner::smoothness_term(int pt_index) {
    Vec2d smooth_term;
    Vec2d current,last,lastlast,next,nextnext;
    if(pt_index>1 && pt_index<base_path_.size()-2) {
        current.set_x(base_path_[pt_index].x());
        current.set_y(base_path_[pt_index].y());
        //if(pt_index>0){
        last.set_x((base_path_[pt_index - 1].x()));
        last.set_y(base_path_[pt_index - 1].y());
        //}
        //if(pt_index>1){
        lastlast.set_y(base_path_[pt_index - 2].y());
        lastlast.set_x(base_path_[pt_index - 2].x());
        //}
        //if(pt_index<base_path_.size()-1){
        next.set_x(base_path_[pt_index + 1].x());
        next.set_y(base_path_[pt_index + 1].y());
        //}
        //if(pt_index<base_path_.size()-2){
        nextnext.set_x(base_path_[pt_index + 2].x());
        nextnext.set_y(base_path_[pt_index + 2].y());
        //}
        smooth_term = 2.0 * (nextnext - 4 * next + 6 * current - 4 * last + lastlast);
        return smooth_term;
    }
    else{
        return smooth_term;
    }
}

Vec2d mm_planner::bound_term(int pt_index) {
    Vec2d bd_term;
    Vec2d base(base_path_[pt_index].x(),base_path_[pt_index].y());
    Vec2d ee(ee_path_[pt_index].x()/map_resolution_,ee_path_[pt_index].y()/map_resolution_);
    double o_bound=outer_bound(ee_path_[pt_index].z()-base_height_)/map_resolution_;
    double i_bound=inner_bound(ee_path_[pt_index].z()-base_height_)/map_resolution_;

    double distance_square=(base.x()-ee.x())*(base.x()-ee.x())+(base.y()-ee.y())*(base.y()-ee.y());
    double distance=sqrt(distance_square);

    //old cost function
    //bd_term=-2*((distance_square-o_bound*o_bound)*2*(base-ee))/pow(distance_square-o_bound*o_bound,4);
    //bd_term+=-2*((distance_square-i_bound*i_bound)*2*(base-ee))/pow(distance_square-i_bound*i_bound,4);

    //new cost function
    double temp= 2*distance-(o_bound+i_bound);
    double cos_data=cos(pi/2*temp*temp/pow((o_bound-i_bound),2));

    bd_term=2*pi*temp*(base-ee)/pow((o_bound-i_bound),2)/distance/pow(cos_data,2);

    //std::cout<<"bound term: "<<bd_term.x()<<" "<<bd_term.y()<<std::endl;
    return bd_term;
}

Vec2d mm_planner::voronoi_term(int pt_index) {
    Vec2d gradient;
    Vec2d xi(base_path_[pt_index].x(),base_path_[pt_index].y());
    float obsDst = voronoi_.getDistance(xi.x(), xi.y());
    Vec2d obsVct(xi.x() - voronoi_.GetClosetObstacleCoor(xi).x(),
                 xi.y() - voronoi_.GetClosetObstacleCoor(xi).y());

    double edgDst = 0.0;
    Vec2i closest_edge_pt = voronoi_.GetClosestVoronoiEdgePoint(xi, edgDst);
    Vec2d edgVct(xi.x() - closest_edge_pt.x(), xi.y() - closest_edge_pt.y());

    if (obsDst < vorObsDMax_ && obsDst > 1e-6) {
        if (edgDst > 0) {
            Vec2d PobsDst_Pxi = obsVct / obsDst;
            Vec2d PedgDst_Pxi = edgVct / edgDst;
//      float PvorPtn_PedgDst = alpha * obsDst * std::pow(obsDst - vorObsDMax, 2) /
//                              (std::pow(vorObsDMax, 2) * (obsDst + alpha) * std::pow(edgDst + obsDst, 2));
            float PvorPtn_PedgDst = (alpha_ / alpha_ + obsDst) *
                                    (pow(obsDst - vorObsDMax_, 2) / pow(vorObsDMax_, 2)) * (obsDst / pow(obsDst + edgDst, 2));

//      float PvorPtn_PobsDst = (alpha * edgDst * (obsDst - vorObsDMax) * ((edgDst + 2 * vorObsDMax + alpha)
//                                                                         * obsDst + (vorObsDMax + 2 * alpha) * edgDst + alpha * vorObsDMax))
//                              / (std::pow(vorObsDMax, 2) * std::pow(obsDst + alpha, 2) * std::pow(obsDst + edgDst, 2));
            float PvorPtn_PobsDst = (alpha_ / (alpha_ + obsDst)) *
                                    (edgDst / (edgDst + obsDst)) * ((obsDst - vorObsDMax_) / pow(vorObsDMax_, 2))
                                    * (-(obsDst - vorObsDMax_) / (alpha_ + obsDst) - (obsDst - vorObsDMax_) / (obsDst + edgDst) + 2);
            gradient = (PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi) * 100;
            return gradient;
        }
        return gradient;
    }
    return gradient;
}

bool mm_planner::is_in_boundary(Point3d ee_pt,Vec2d base_pos) {

    double i_bound = inner_bound(ee_pt.z()-base_height_)/map_resolution_;
    double o_bound = outer_bound(ee_pt.z()-base_height_)/map_resolution_;
    double distance=hypot(ee_pt.x()/map_resolution_-base_pos.x(),ee_pt.y()/map_resolution_-base_pos.y());
    //std::cout<<"distance: "<<distance<<" inner: "<<i_bound<<" outer: "<<o_bound<<std::endl;
    if (distance < o_bound && distance > i_bound){
        return true;
    }
    else{
        //std::cout<<"distance: "<<distance<<" inner: "<<i_bound<<" outer: "<<o_bound<<std::endl;
        return false;
    }

}

void mm_planner::print_vector_pose2d(std::vector<Pose2d> data) {
    for(auto pose:data){
        std::cout<<pose.x()<<" "<<pose.y()<<std::endl;
    }
}

void mm_planner::show_final_path() {
    //std::cout<<"original path: "<<std::endl;
    //print_vector_pose2d(base_origin_path_);

    cv::Mat map = cv::Mat::zeros(cv::Size(map_width_, map_height_), CV_8UC1);
    map=map_img_;
    if(map.channels() != 1) {
        cv::cvtColor(map, map, cv::COLOR_BGR2GRAY);
    }

    for(int i=0;i<base_origin_path_.size()-1;i++){
        cv::Point cur(base_origin_path_[i].x(),base_origin_path_[i].y());
        cv::Point next(base_origin_path_[i+1].x(),base_origin_path_[i+1].y());
        cv::line(map,cur,next,cv::Scalar(0));
        cv::circle(map,cur,5,cv::Scalar(1));
        //imshow("map", map);
        //cv::waitKey();
    }

    for(int i=0;i<ee_path_.size()-1;i++){
        cv::Point cur(ee_path_[i].x()/map_resolution_,ee_path_[i].y()/map_resolution_);
        cv::Point next(ee_path_[i+1].x()/map_resolution_,ee_path_[i+1].y()/map_resolution_);
        cv::line(map,cur,next,cv::Scalar(0));
        //imshow("map", map);
        //cv::waitKey();
    }

    imshow("original path", map);
    cv::waitKey();

    //std::cout<<"optimized path: "<<std::endl;
    //print_vector_pose2d(base_path_);

    for(int i=0;i<base_path_.size()-1;i++) {
        cv::Point cur(base_path_[i].x(), base_path_[i].y());
        cv::Point next(base_path_[i + 1].x(), base_path_[i + 1].y());
        cv::line(map, cur, next, cv::Scalar(0));
        cv::circle(map, cur, 3, cv::Scalar(1));
    }
    imshow("optimized path", map);
    cv::waitKey();
    imwrite("../map_imgs/optimized_path.png", map);
}