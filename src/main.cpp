#include "control_points.hpp"
#include "correspondences.hpp"
#include "ls_solver.hpp"


#include "preprocess.hpp"

int main() {
    // Load the image   
    cv::String folder = "/Users/amtouti/Documents/SFM/3d_sample_data_set_sofa/3D_sofas/";
    std::vector<MouseCallbackParams*> control_points;
    std::vector<cv::Point3f> XYZ_control_points;
    ControlPoints processor;

    bool data_ready = 1;
    bool debug = 0;
    bool get_more_correspondences = 0;
    processor.processControlPoints(folder, control_points, XYZ_control_points, data_ready); 

    if(debug){
        processor.printControlPixels(control_points);
        processor.printControlXYZ(XYZ_control_points);
    }

    cv::Mat img1 = cv::imread("/Users/amtouti/Documents/SFM/3d_sample_data_set_sofa/3D_sofas/IMG_1838.jpeg");
    cv::Mat img2 = cv::imread("/Users/amtouti/Documents/SFM/3d_sample_data_set_sofa/3D_sofas/IMG_1839.jpeg");

    std::vector<Correspondence> Corr;

    GetCorrespondence Correspond(img1, img2);
    Correspond.readCorrFromFile(Corr);
    
    if(get_more_correspondences){
        Correspond.getCorrespondence(Corr);
            std::cout << "Values After getting correspondences" << std::endl;
        for(const auto& corr: Corr){
            std::cout << corr.p1 << " " << corr.p2 << std::endl;
        }
        Correspond.writeCorrToFile(Corr);

    }

    MouseCallbackParams* last_image = new MouseCallbackParams;
    std::vector<cv::Point2i> control_points_pixel;

    last_image = control_points.back();
    for(const auto& control_pixel: last_image->dataPoints){
        control_points_pixel.push_back(control_pixel.pixelLocation);
        // std::cout << control_pixel.pixelLocation << std::endl;
    }

    std::vector<cv::Point2d> correspondence_p1;
    std::vector<cv::Point2d> correspondence_p2;

    for(const auto& corr: Corr){
        correspondence_p1.push_back(corr.p1);
        correspondence_p2.push_back(corr.p2);
    }

    cv::destroyAllWindows();
    
    double fx = 2400.0; 
    double fy = 2400.0; 
    double cx = 2016.0; 
    double cy = 1512.0; 
    double skew = 0.0; 

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
                 fx, skew, cx,
                 0, fy, cy,
                 0, 0, 1);


    cv::Mat essential_matrix =  cv::findEssentialMat(correspondence_p1, correspondence_p2, K, 
                                                     cv::RANSAC, 0.999, 3.0, 1000);	

    cv::Mat R, t;
    int inliers = cv::recoverPose(essential_matrix, correspondence_p1, correspondence_p2, K ,R, t);

    std::cout << "NUMBER OF INLIERS IN RECIVER POSE IS: " << inliers << std::endl;

    cv::Mat transformationMatrix_2 = cv::Mat::zeros(3, 4, CV_64F);
    
    R.copyTo(transformationMatrix_2(cv::Rect(0, 0, 3, 3)));

    t.copyTo(transformationMatrix_2(cv::Rect(3, 0, 1, 3)));

    cv::Mat projecMatrix_2 = K * transformationMatrix_2;

    cv::Mat transformationMatrix_1 = (cv::Mat_<double>(3,4) <<
                                        1, 0, 0, 0,
                                        0, 1, 0, 0,
                                        0, 0, 1, 0);


    cv::Mat projecMatrix_1 = K * transformationMatrix_1;
    cv::Mat world_points;

    cv::triangulatePoints(projecMatrix_1, projecMatrix_2, correspondence_p1, correspondence_p2, world_points);

    std::vector<cv::Mat> homogeneous_points;
    std::vector<cv::Point3f> disp_points;
    std::vector<cv::Vec3b> colors;

    getHomogneousPoints(world_points, Corr,  img2, homogeneous_points, disp_points, colors);

    // visualise(disp_points,colors,transformationMatrix_1, transformationMatrix_2, K);

    LS_SOLVER ls_solver(Corr, homogeneous_points, transformationMatrix_1, transformationMatrix_2, K);
    ls_solver.solveLS();

    std::vector<cv::Point3f> disp_points_;

    getLSPoints(ls_solver.x, disp_points_);

    visualise(disp_points_,colors,transformationMatrix_1, ls_solver.transformation_matrix_2, ls_solver.K);
    
    return 0;
}