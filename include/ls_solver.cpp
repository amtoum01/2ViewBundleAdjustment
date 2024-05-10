#include "ls_solver.hpp"


cv::Point2d LS_SOLVER::get_err(cv::Point2d p, cv::Mat transformation_matrix, cv::Mat K, cv::Mat hom_XYZ){


    cv::Mat p_est = K * transformation_matrix * hom_XYZ;
    p_est /= p_est.at<double>(2);
    cv::Point2d p_est_;
    p_est_.x = p_est.at<double>(0);
    p_est_.y = p_est.at<double>(1);
    cv::Point2d diff = p - p_est_;
    return diff;


}

double LS_SOLVER::total_error_func(cv::Mat& residual){
    double err = 0;
    cv::Mat p1;
    cv::Mat p2;
    double err_1;
    double err_2;
    
    int split = 2*num_points;

    cv::Point2d p1_err;
    cv::Point2d p2_err;
    for(int i=0; i<num_points; i++){

        p1_err = get_err(Corr[i].p1, transformation_matrix_1, K, hom_XYZ[i]);
        p2_err = get_err(Corr[i].p2, transformation_matrix_2, K, hom_XYZ[i]);

        residual.at<double>(2*i) = p1_err.x;
        residual.at<double>(2*i+1) = p1_err.y;

        residual.at<double>(2*i + split) = p2_err.x;
        residual.at<double>(2*i + split + 1) = p2_err.y;

        err_1 = (std::pow(p1_err.x,2) + std::pow(p1_err.y,2));
        err_2 = (std::pow(p2_err.x,2) + std::pow(p2_err.y,2));

        err += std::sqrt(err_1) + std::sqrt(err_2);
    }

    std::cout << "TOTAL ERROR IS: " << err << std::endl;
    std::cout << std::endl;

    return err;
}

void LS_SOLVER::rotationToRodriguez(cv::Mat& rotation_mat, cv::Mat& rodriguez_vec){
    cv::Rodrigues(rotation_mat, rodriguez_vec);

}

void LS_SOLVER::rodriguezToRotation(cv::Mat& rodriguez_vec, cv::Mat& rotation_mat){
    cv::Rodrigues(rodriguez_vec, rotation_mat);
}

void LS_SOLVER::exportToCSV(cv::Mat& matrix, std::string& filename) {
    std::ofstream outputFile(filename);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
        return;
    }

    for (int i = 0; i < matrix.rows; ++i) {
        for (int j = 0; j < matrix.cols; ++j) {
            outputFile << matrix.at<double>(i, j);
            if (j < matrix.cols - 1) {
                outputFile << ","; // Separate values with comma
            }
        }
        outputFile << std::endl; // Move to the next line after each row
    }

    outputFile.close();
}

void LS_SOLVER::resetJacob(){
    jacob = cv::Mat::zeros(4*num_points, no_params, CV_64F);
}

void LS_SOLVER::computeJacob(){

    cv::Mat K_cp;
    cv::Mat hom_XYZ_cp;
    cv::Point2d err_1_mod;
    cv::Point2d err_2_mod;
    cv::Point2d deriv_1;
    cv::Point2d deriv_2;
    cv::Mat rot_2;
    cv::Mat rod_2;
    cv::Mat rod_2_cp;
    cv::Mat rot_2_cp;
    cv::Rect rot_roi(0, 0, 3, 3);
    transformation_matrix_2(rot_roi).copyTo(rot_2);
    rotationToRodriguez(rot_2, rod_2);
    cv::Mat transformation_matrix_1_cp;
    cv::Mat transformation_matrix_2_cp;
    transformation_matrix_1.copyTo(transformation_matrix_1_cp);
    transformation_matrix_2.copyTo(transformation_matrix_2_cp);
    cv::Mat rot_cp;
    cv::Mat jacob_rod;

    double delta = 0.001;
    int jacop_split = 2 * num_points;

    for(int i=0; i<Corr.size(); i++){
        cv::Point2d err_1 = get_err(Corr[i].p1, transformation_matrix_1, K, hom_XYZ[i]);
        cv::Point2d err_2 = get_err(Corr[i].p2, transformation_matrix_2, K, hom_XYZ[i]);
        
        for(int o=0; o<4; o++){
            K.copyTo(K_cp);
            K_cp.at<double>(K_ind[o].first, K_ind[o].second) += delta;
            err_1_mod = get_err(Corr[i].p1, transformation_matrix_1, K_cp, hom_XYZ[i]);
            err_2_mod = get_err(Corr[i].p2, transformation_matrix_2, K_cp, hom_XYZ[i]);
            deriv_1 = (err_1_mod - err_1) / delta;
            deriv_2 = (err_2_mod - err_2) / delta;
            jacob.at<double>(2*i, o) = deriv_1.x;
            jacob.at<double>(2*i+1, o) = deriv_1.y;
            jacob.at<double>(2*i + jacop_split, o) = deriv_2.x;
            jacob.at<double>(2*i + jacop_split + 1, o) = deriv_2.y;
        }

        for(int k=0; k<3; k++){
            transformation_matrix_2.copyTo(transformation_matrix_2_cp);
            rod_2.copyTo(rod_2_cp);
            rod_2_cp.at<double>(k) += delta;
            cv::Rodrigues(rod_2_cp, rot_2_cp);
            rot_2_cp.copyTo(transformation_matrix_2_cp(rot_roi));
            err_2_mod = get_err(Corr[i].p2, transformation_matrix_2_cp, K, hom_XYZ[i]);
            deriv_2 = (err_2_mod - err_2) / delta;
            jacob.at<double>(2*i + jacop_split, ext_2_startidx + k) = deriv_2.x;
            jacob.at<double>(2*i + + jacop_split + 1, ext_2_startidx + k) = deriv_2.y;
        }

        for(int p=1; p<3; p++){
            transformation_matrix_2.copyTo(transformation_matrix_2_cp);
            transformation_matrix_2_cp.at<double>(p,3) += delta;
            err_2_mod = get_err(Corr[i].p2, transformation_matrix_2_cp, K, hom_XYZ[i]);
            deriv_2 = (err_2_mod - err_2) / delta;
            jacob.at<double>(2*i + jacop_split, ext_2_startidx  + 3 + p - 1) = deriv_2.x;
            jacob.at<double>(2*i + jacop_split + 1, ext_2_startidx + 3 + p - 1) = deriv_2.y;

        }

        for(int j=0; j<3; j++){
            hom_XYZ[i].copyTo(hom_XYZ_cp);
            hom_XYZ_cp.at<double>(j) += delta;
            err_1_mod = get_err(Corr[i].p1, transformation_matrix_1, K, hom_XYZ_cp);
            err_2_mod = get_err(Corr[i].p2, transformation_matrix_2, K, hom_XYZ_cp);
            deriv_1 = (err_1_mod - err_1) / delta;
            deriv_2 = (err_2_mod - err_2) / delta;

            
            jacob.at<double>(2*i, hom_pts_startidx + 3*i + j) = deriv_1.x;
            jacob.at<double>(2*i + 1, hom_pts_startidx + 3*i + j) = deriv_1.y;

            jacob.at<double>(2*i + jacop_split, hom_pts_startidx + 3*i + j) = deriv_2.x;
            jacob.at<double>(2*i + jacop_split + 1, hom_pts_startidx + 3*i + j) = deriv_2.y;
        }
    }
}

void LS_SOLVER::getStateVec(){
    
    std::vector<std::pair<int,int>> K_ind = {{0,0},{1,1},{0,2},{1,2}};
   
    for(int i=0; i<4; i++){
        x.at<double>(i) = K.at<double>(K_ind[i].first, K_ind[i].second);
    }

    cv::Rect rot_roi(0, 0, 3, 3);
    cv::Mat rot_2 = transformation_matrix_2(rot_roi);
    cv::Mat rod_2;
    cv::Rodrigues(rot_2, rod_2);

    for(int j=0; j<3; j++){
        x.at<double>(ext_2_startidx + j) = rod_2.at<double>(j);
    }

    for(int k=1; k<3; k++){
        x.at<double>(ext_2_startidx + 3 + k - 1) = transformation_matrix_2.at<double>(k, 3);
    }
    
    for(int u=0; u<hom_XYZ.size(); u++){
        x.at<double>(hom_pts_startidx + 3*u) = hom_XYZ[u].at<double>(0);
        x.at<double>(hom_pts_startidx + 3*u+1) = hom_XYZ[u].at<double>(1);
        x.at<double>(hom_pts_startidx + 3*u+2) = hom_XYZ[u].at<double>(2);
    }
}

void LS_SOLVER::stateVecToMat(){
    

    for(int i=0; i<4; i++){
        K.at<double>(K_ind[i].first, K_ind[i].second) = x.at<double>(i);
    }

    cv::Rect rot_roi(0, 0, 3, 3);
    cv::Mat rot_2;
    cv::Mat rod_2 = x.rowRange(4,7);

    cv::Rodrigues(rod_2, rot_2);

    rot_2.copyTo(transformation_matrix_2(rot_roi));


    for(int i=1; i<3; i++){
        transformation_matrix_2.at<double>(i,3) = x.at<double>(ext_2_startidx + 3 + i - 1);
    }

    for(int i=0; i<hom_XYZ.size(); i++){
        hom_XYZ[i].at<double>(0) = x.at<double>(hom_pts_startidx + 3*i);
        hom_XYZ[i].at<double>(1) = x.at<double>(hom_pts_startidx + 3*i + 1);
        hom_XYZ[i].at<double>(2) = x.at<double>(hom_pts_startidx + 3*i + 2);
    }

}

void LS_SOLVER::solveLS(){

    x = cv::Mat(no_params,1,CV_64F);
    cv::Mat Hess;
    cv::Mat residuals = cv::Mat::zeros(4*num_points, 1, CV_64F);
    double err;

    for(int i=0; i<no_iter; i++){
        std::cout << " Iter: " << i << std::endl;
        err = total_error_func(residuals);
        resetJacob();
        computeJacob();
        std::cout << "Jacob rows " << jacob.rows << " Jacob cols " << jacob.cols << std::endl;
        Hess = jacob.t() * jacob;
        cv::solve(Hess, -jacob.t() * residuals, delta, cv::DECOMP_SVD);
        getStateVec();
        x += delta;
        stateVecToMat();

    }
    
}

LS_SOLVER::LS_SOLVER(std::vector<Correspondence> Corr, std::vector<cv::Mat> hom_XYZ, cv::Mat transformation_matrix_1, cv::Mat transformation_matrix_2, cv::Mat K):
    Corr(Corr), hom_XYZ(hom_XYZ), transformation_matrix_1(transformation_matrix_1), transformation_matrix_2(transformation_matrix_2), K(K){
        num_points = Corr.size();
        no_params = 4 + 5 + 3*num_points;
    } 