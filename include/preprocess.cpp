#include "preprocess.hpp"

void getHomogneousPoints(const cv::Mat& world_points, const std::vector<Correspondence>& Corr, const cv::Mat& img, std::vector<cv::Mat>& homogeneous_points, std::vector<cv::Point3f>& disp_points, std::vector<cv::Vec3b>& colors){
    
    int num_points = world_points.cols;

    cv::Point3f disp_point;
    homogeneous_points.clear();
    disp_points.clear();
    colors.clear();
    
    

    for (int i = 0; i < num_points; ++i) {
        
        colors.push_back(img.at<cv::Vec3b>(Corr[i].p2.y, Corr[i].p2.x));

        cv::Vec4d homogeneous_point = cv::Vec4d(world_points.col(i).at<double>(0),
                                                 world_points.col(i).at<double>(1),
                                                 world_points.col(i).at<double>(2),
                                                 world_points.col(i).at<double>(3));
        homogeneous_point /= world_points.col(i).at<double>(3);
        cv::Mat hom_xyz(homogeneous_point);
        homogeneous_points.push_back(hom_xyz);
        disp_point.x = world_points.col(i).at<double>(0);
        disp_point.y = world_points.col(i).at<double>(1);
        disp_point.z = world_points.col(i).at<double>(2);
        disp_points.push_back(disp_point);
    }
}

void getLSPoints(const cv::Mat& x, std::vector<cv::Point3f>& disp_points){

    cv::Point3f disp_point;
    disp_points.clear();
    int num_points = (x.rows - 9) / 3;

    for (int i = 0; i < num_points; i++) {

        disp_point.x = x.at<double>(9 + 3*i);
        disp_point.y = x.at<double>(9 + 3*i + 1);
        disp_point.z = x.at<double>(9 + 3*i + 2);
        disp_points.push_back(disp_point);
    }
}

void visualise(const std::vector<cv::Point3f>& disp_points, const std::vector<cv::Vec3b>& colors, const cv::Mat& transformation_matrix_1, const cv::Mat& transformation_matrix_2, const cv::Mat& K){

    cv::Mat R_2, t_2;
    R_2 = transformation_matrix_2(cv::Rect(0, 0, 3, 3)).t();
    t_2 = transformation_matrix_2(cv::Rect(3, 0, 1, 3));

    cv::Matx33d R_matx2(R_2);
    cv::Vec3d t_vec2(t_2);
    cv::Affine3d pose2(R_matx2, -t_vec2);


    cv::Matx33d K_f(K);

    cv::viz::WCameraPosition frust1(K_f, 0.3, cv::viz::Color::white());
    cv::viz::WCameraPosition frust2(K_f, 0.3, cv::viz::Color::white());

    
 
    cv::viz::Viz3d window("Point Cloud");
    cv::Mat points_mat(disp_points);
    cv::Mat colors_mat(colors);    

    cv::viz::WCloud cloud_widget(points_mat, colors_mat);
    cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, 10.0);
    window.showWidget("Point Cloud", cloud_widget);
    window.showWidget("Frustum 1", frust1);
    window.showWidget("Frustum 2", frust2);
    window.setWidgetPose("Frustum 2", pose2);
    while (!window.wasStopped()) {
        window.spinOnce(1, true);
    }
}