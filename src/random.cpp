

#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    // Read the image

    std::vector<cv::String> filenames;
    cv::String folder = "/Users/amtouti/Documents/SFM/3d_sample_data_set_sofa/3D sofas/";
    cv::glob(folder, filenames);

    // Convert the image to grayscale
    

    for(const auto& file: filenames){

        cv::Mat image = cv::imread(file);
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        cv::imshow("Result", gray);
        cv::waitKey(0);
        int bin_thresh = 150;

        if(file[68] == '3' && file[69] == '9'){
            bin_thresh = 130;
            std::cout << "okayyy" << std::endl;

        }
        std::cout << "BINARY THRESH" << bin_thresh << std::endl;
        // Apply thresholding to binarize the image
        cv::Mat binary;
        cv::threshold(gray, binary, bin_thresh, 255, cv::THRESH_BINARY);
        cv::imshow("Result", binary);
        cv::waitKey(0);

        cv::Mat canny;

        cv::Canny(gray, canny, 50, 150, 3);

        cv::imshow("Result", canny);
        cv::waitKey(0);

        // cv::bitwise_not(binary, binary);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(canny, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        // for (const auto& c: contours){
        //     std::cout << cv::contourArea(c) << " " << c.size() << std::endl;
        // }

        std::vector<cv::Moments> mu(contours.size());
        for( int i = 0; i<contours.size(); i++ ){
             mu[i] = cv::moments( contours[i], false );
        }
        std::vector<cv::Point2f> mc(contours.size());
        for( int i = 0; i<contours.size(); i++){
             mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
        }

        cv::Mat drawing(canny.size(), CV_8UC3, cv::Scalar(255,255,255));
        std::vector<cv::Vec4i> hierarchy;
        for( int i = 0; i<contours.size(); i++ ){
            std::vector<cv::Point> approx;
            double perimeter = cv::arcLength(contours[i], true);
            cv::approxPolyDP(contours[i], approx, 1.0 * perimeter, true);
            if(approx.size() < 2 && contours[i].size() > 100 && cv::contourArea(contours[i]) > 30){
                cv::Scalar color = cv::Scalar(167,151,0); // B G R values
                drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
                circle( drawing, mc[i], 4, color, -1, 8, 0 );

            }
            
        }
        
        // show the resultant image
        cv::namedWindow( "Contours", cv::WINDOW_AUTOSIZE );
        cv::imshow( "Contours", drawing );
        cv::waitKey(0);


        

        // Filter contours to find the contour of the plus sign
        std::vector<cv::Point> plus_sign_contour;
        for (const auto& contour : contours) {
            std::cout << contour << std:: endl;
            // Filter by contour area and number of vertices
            if (contour.size() < 5 ) { // Adjust threshold and contour size
                plus_sign_contour = contour;
                break;
            }
        }

        std::vector<std::vector<cv::Point>> conts;
        conts.push_back(contours[0]);

        std::cout << plus_sign_contour << std::endl;
        // for(const auto& point: plus_sign_contour){
        //     std::cout << point << std::endl;
        // }

        // std::cout << plus_sign_contour << std::endl;

        cv::Moments moments = cv::moments(plus_sign_contour);

        cv::Point centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);

        std::cout << centroid << std:: endl;

        cv::drawContours(image, contours, -1, cv::Scalar(0, 255, 0), 2);
        cv::circle(image, centroid, 100, cv::Scalar(0, 0, 255), -1);

        // Display the result
        cv::imshow("Result", image);
        cv::waitKey(0);
        cv::destroyAllWindows();

    }
    

    // cv::GaussianBlur(gray, gray, cv::Size(11, 11), 0);
    

    return 0;
}

