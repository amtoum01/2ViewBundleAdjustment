#ifndef CONTROL_POINTS
#define CONTROL_POINTS

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

struct DataPoint {
    cv::Point2i pixelLocation;
    int number;
};

struct MouseCallbackParams {
    cv::Mat image;
    std::vector<DataPoint> dataPoints;
    cv::String img_num;
};

class ControlPoints
{
private:
    static void onMouse(int event, int x, int y, int flags, void* userdata);
    void writeToFile(std::vector<MouseCallbackParams*> control_points);
    void readFromFile(std::vector<MouseCallbackParams*>& control_points);
    void readXYZFromFile(std::vector<cv::Point3f>& XYZ_control_points);
    
public:
    
    void processControlPoints(const cv::String& folder, std::vector<MouseCallbackParams*>& control_points,
                              std::vector<cv::Point3f>& XYZ_control_points, int data_ready);
    void printControlPixels(std::vector<MouseCallbackParams*>& control_points);
    void printControlXYZ(std::vector<cv::Point3f>& XYZ_control_points);

};

#endif 
