#ifndef ICP_H
#define ICP_H

#include <iostream>
#include <vector> 
#include <mutex>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "tracker/parameter.hpp"

class ICP
{
private:
    double degree2radian;

private:
    void transform(cv::Mat& from, cv::Mat& to, allen::Frame frame);
    double getPointDist(cv::Mat& data, int idx1, int idx2);
    void centroid(cv::Mat& target, cv::Mat& center);
    void match(cv::flann::Index& flann_idx, cv::Mat& from, cv::Mat& to, 
            cv::Mat& from_inlier, cv::Mat& to_inlier, double ratio, cv::Mat& draw);

public:
    cv::Mat from_inlier_;

public:
    ICP(/* args */)
    {
        degree2radian = (double)M_PI / 180.0;
    }
    ~ICP()
    {}
    bool run(cv::Mat &from, cv::Mat &to, allen::Frame &output, cv::flann::Index &flann_idx, cv::Mat &draw);
    void reset()
    {
        this->from_inlier_ = cv::Mat();
    }

};

#endif