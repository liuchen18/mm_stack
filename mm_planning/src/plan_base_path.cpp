#include <iostream>
#include <vector>
#include "pose2d.h"
#include "point3d.h"
#include "mm_planner.h"
#include <cmath>
#include <fstream>
#define pi 3.1415926
/*
///broken line
std::vector<Point3d> generate_ee_data(){
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    for(int i=0;i<250;i++){
        z.push_back(800);
        x.push_back(i*20+3000);
        if(i<50){
            y.push_back(i*20+3800);
        }
        else if(i<150){
            y.push_back(-i*20+5800);
        }
        else{
            y.push_back(i*20-200);
        }
    }

    std::vector<Point3d> ee_data;
    for(int i=0;i<x.size();i++){
        ee_data.emplace_back(x[i],y[i],z[i]);
    }
    return ee_data;

}
*/

///sin
std::vector<Point3d> generate_ee_data(){
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    for(int i=0;i<300;i++){
        z.push_back(1300);
        x.push_back(i*20+900);
        y.push_back(1000*sin(i*1.0/90*pi)+4000);
    }

    std::ofstream myout("../data/ee_path.txt");
    for(int i=0;i<x.size();i++){
        myout<<i*1.0<<" "<<x[i]<<" "<<y[i]<<" "<<z[i]<<std::endl;
    }
    std::vector<Point3d> ee_data;
    for(int i=0;i<x.size();i++){
        ee_data.emplace_back(x[i],y[i],z[i]);
    }
    return ee_data;

}


void write_result_to_txt(std::vector<Pose2d> base_pose){
    std::ofstream myout("../data/base_path.txt");
    for(int i=0;i<base_pose.size();i++){
        myout<<i<<" "<<base_pose[i].x()*10<<" "<<base_pose[i].y()*10<<" "<<0<<std::endl;
    }
}

int main() {

    std::vector<Point3d> ee_data=generate_ee_data();

    auto img = cv::imread("../map_imgs/map.png", 0);


    if(img.empty()) {
        std::cout << "load map image failed" << std::endl;
        return -1;
    }

    mm_planner planner(img,ee_data);
    planner.set_max_iteration(1000);
    planner.plan();
    std::vector<Pose2d> base_path=planner.get_base_path();

    write_result_to_txt(base_path);

    return 0;
}
