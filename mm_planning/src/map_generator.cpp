//
// Created by hilsys on 2021/1/20.
//
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/core.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "vec2d.h"

//using namespace std;
void generate_map(std::vector<std::vector<int>> obs_list) {
    int height = 700, width = 1000;
    cv::Mat map_img = cv::Mat::zeros(cv::Size(width, height), CV_8UC1);
    for(int i=0;i<height;i++){
        for(int j=0;j<width;j++){
            if(i<10 || i> height-10 || j<10 || j>width-10){
                map_img.at<uchar>(i,j)=0;
            }
            else {
                map_img.at<uchar>(i, j) = 255;
            }
        }
    }

    for(auto obs:obs_list){
        for(int m=std::max(0,obs[0]-obs[2]);m<std::min(width,obs[0]+obs[2]);m++){
            for(int n=std::max(0,obs[1]-obs[2]);n<std::min(height,obs[1]+obs[2]);n++){
                map_img.at<uchar>(n,m)=0;
            }
        }
    }



    imshow(" map", map_img);

    cv::waitKey();
    imwrite("../map_imgs/map.png", map_img);

}

int main() {
    std::vector<std::vector<int>> obs_list;
    std::vector<int> obs1={400,400,20};
    std::vector<int> obs2={600,300,20};
    obs_list.push_back(obs1);
    obs_list.push_back(obs2);

    generate_map(obs_list);

}