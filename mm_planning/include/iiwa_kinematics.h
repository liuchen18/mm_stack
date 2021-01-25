//
// Created by hilsys on 2021/1/25.
//

#ifndef MM_PLANNING_IIWA_KINEMATICS_H
#define MM_PLANNING_IIWA_KINEMATICS_H

#include "kinematics.h"

/**
 * compute the forward kinematics for iiwa14
 * @param results
 * @param joint_values
 */
void forward_kinematics(std::vector<double>& results,std::vector<double> joint_values){

}

/**
 * compute the inverse kinematics for iiwa14
 * @param next_joint_values the results of current computation
 * @param ee given end effector pose and position, in euler angle
 * @param phi arm angle, default to 0
 * @param current_joint_values current joint values,
 * @return null
 */
void inverse_kinematics(std::vector<double>& next_joint_values,
                        std::vector<double> ee,
                        std::vector<double> current_joint_values,
                        double phi){
    if(current_joint_values.size()!=7 || next_joint_values.size()!=7){
        std::cout<<"invalid input for joint values"<<std::endl;
        exit(0);
    }
    double next[7],pre[7];
    cv::Mat ee_data=cv::Mat::zeros(6,1,CV_32FC1);
    for(int i=0;i<7;i++){
        pre[i]=current_joint_values[i];
        ee_data.ptr<float>(0)[i]=ee[i];
    }
    Inv_Kine(next,ee_data,phi,pre,7);
    for(int i=0;i<7;i++){
        next_joint_values[i]=next[i];
    }

}

/**
 * compute the inverse kinematics for iiwa 14
 * @param ee given end effector pose and position, in euler angle
 * @param current_joint_values
 * @param phi arm angle, default to 0
 * @return std::vector<double>, the results of the current computation
 */
std::vector<double> inverse_kinematics(std::vector<double> ee,
                                       std::vector<double> current_joint_values,
                                       double phi=0){
    if(current_joint_values.size()!=7){
        std::cout<<"invalid input for joint values"<<std::endl;
        exit(0);
    }
    double next[7],pre[7];
    cv::Mat ee_data=cv::Mat::zeros(6,1,CV_32FC1);
    for(int i=0;i<7;i++){
        pre[i]=current_joint_values[i];
        ee_data.ptr<float>(0)[i]=ee[i];
    }
    Inv_Kine(next,ee_data,phi,pre,7);

    std::vector<double> next_joint_values(7,0);
    for(int i=0;i<7;i++){
        next_joint_values[i]=next[i];
    }
    return next_joint_values;
}

#endif //MM_PLANNING_IIWA_KINEMATICS_H
