//
// Created by hilsys on 2021/1/25.
//
#include "iiwa_kinematics.h"
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

std::vector<std::vector<double>> get_data(std::string dir,int num_in_line){
    ifstream file(dir);
    std::vector<std::vector<double>> data;

    if(!file.is_open()){
        std::cout<<"can not open this file"<<std::endl;
        return data;
    }
    std::string line;
    double temp;
    while(file>>temp){
        std::vector<double> temp_vector={temp};
        for(int i=0;i<num_in_line-1;i++){
            file>>temp;
            temp_vector.push_back(temp);
        }
        data.push_back(temp_vector);
    }
    return data;

}

void write_result_to_txt(std::vector<std::vector<std::vector<double>>> joint_values){
    std::ofstream myout("../data/manipulator_path.txt");
    for(auto joint:joint_values){
        myout<<joint[0][0]<<" "<<joint[1][0]<<" "<<joint[1][1]<<" "<<joint[1][2]<<" "<<
             joint[1][3]<<" "<<joint[1][4]<<" "<<joint[1][5]<<" "<<joint[1][6]<<std::endl;
    }
}


int main(){
    string ee_dir="../data/ee_path.txt";
    string base_dir="../data/base_path.txt";
    std::vector<std::vector<double>> ee_path=get_data(ee_dir,4);
    std::vector<std::vector<double>> base_path=get_data(base_dir,4);
    std::vector<std::vector<vector<double>>> manioulator_path;

    for (int i=0;i<ee_path.size();i++){
        std::vector<std::vector<double>> ee;
        std::vector<double> ee_time={ee_path[i][0]};
        ee.push_back(ee_time);
        std::vector<double> ee_position(7,0);
        //compute x
        ee_position[0]=ee_path[i][1]-base_path[i][1];
        //compute y
        ee_position[1]=ee_path[i][2]-base_path[i][2];
        ee_position[2]=ee_path[i][3]-400;
        ee.push_back(ee_position);
        manioulator_path.push_back(ee);

    }
    std::vector<std::vector<std::vector<double>>> joint_values;
    std::vector<double> current_joint_value={-0.05475,-0.46492,0,-1.85293,0,-1.49258,-0.054};
    for(auto ee: manioulator_path){
        std::vector<double> ik_result=inverse_kinematics(ee[1],current_joint_value,0);
        std::vector<std::vector<double>> cur_joint;
        std::vector<double> joint_time={ee[0][0]};
        cur_joint.push_back(joint_time);
        cur_joint.push_back(ik_result);
        joint_values.push_back(cur_joint);
    }

    write_result_to_txt(joint_values);



    return 0;
}
