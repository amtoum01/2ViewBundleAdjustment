#ifndef LSSOLVER
#define LSSOLVER

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "correspondences.hpp"

class LS_SOLVER
{
private:
    int ext_2_startidx = 4;
    int hom_pts_startidx = 4 + 5;
    std::vector<Correspondence> Corr;
    std::vector<std::pair<int,int>> K_ind = {{0,0},{1,1},{0,2},{1,2}};

    int num_points;
    int no_params;
    int no_iter = 5;

    cv::Mat jacob;
    cv::Mat delta;
    
    cv::Point2d get_err(cv::Point2d p, cv::Mat transformation_matrix, cv::Mat K, cv::Mat hom_XYZ);
    double total_error_func(cv::Mat& residual);
    void rotationToRodriguez(cv::Mat& rotation_mat, cv::Mat& rodriguez_vec);
    void rodriguezToRotation(cv::Mat& rodriguez_vec, cv::Mat& rotation_mat);
    void exportToCSV(cv::Mat& matrix,  std::string& filename);
    void resetJacob();
    void computeJacob();
    void getStateVec();
    void stateVecToMat();
    
public:

    std::vector<cv::Mat> hom_XYZ;
    cv::Mat transformation_matrix_1;
    cv::Mat transformation_matrix_2;
    cv::Mat K;
    cv::Mat x;

    LS_SOLVER(std::vector<Correspondence> Corr, std::vector<cv::Mat> hom_XYZ, cv::Mat transformation_matrix_1, cv::Mat transformation_matrix_2, cv::Mat K);
    
    void solveLS();
};


#endif