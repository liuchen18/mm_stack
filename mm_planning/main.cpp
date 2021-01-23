#include <iostream>
#include <vector>
#include "pose2d.h"
#include "point3d.h"
#include "mm_planner.h"

std::vector<Point3d> generate_ee_data(){
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    for(int i=0;i<200;i++){
        z.push_back(800);
        x.push_back(i*20+3000);
        if(i<50){
            y.push_back(i*20+2000);
        }
        else if(i<150){
            y.push_back(-i*20+4000);
        }
        else{
            y.push_back(i*20-2000);
        }
    }

    std::vector<Point3d> ee_data;
    for(int i=0;i<x.size();i++){
        ee_data.emplace_back(x[i],y[i],z[i]);
    }
    return ee_data;

}

int main() {

    std::vector<Point3d> ee_data=generate_ee_data();

    auto img = cv::imread("../map_imgs/map.png", 0);


    if(img.empty()) {
        std::cout << "load map image failed" << std::endl;
        return -1;
    }

    mm_planner planner(img,ee_data);
    planner.set_max_iteration(200);
    planner.plan();
    std::vector<Pose2d> base_path=planner.get_base_path();


    return 0;
}
