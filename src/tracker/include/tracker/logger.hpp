#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <mutex>
#include <ctime>
#include <string>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

class logger
{
public:
    std::string file_name, dst, cur_time, format_name;
    cv::FileStorage::Mode mode;
    cv::FileStorage file;

private:
    void initialize()
    {
        std::string full_dst;
        //get curTime
        struct tm cur_tm; 
        time_t curr_time = time(nullptr);
        localtime_r(&curr_time, &cur_tm);
        int cur_hour = cur_tm.tm_hour;
        int cur_min = cur_tm.tm_min;
        std::string tmp_h_s = std::to_string(cur_hour);
        std::string tmp_m_s = std::to_string(cur_min);
        cur_time = tmp_h_s + tmp_m_s;

        full_dst = dst + file_name + "_" + cur_time + format_name;
        file = cv::FileStorage(full_dst, mode);
        if(!file.isOpened())
        {
            std::cerr << "File open failed!" << std::endl;
            return;
        }
        printf("FILE[%s] is ready for %d in %s!\n", file_name.c_str(), mode, dst.c_str());
    }

public:
    logger(){}
    logger(std::string _dst, std::string _file_name, cv::FileStorage::Mode _mode)  :
        file_name(_file_name), mode(_mode), dst(_dst), format_name(".yml")
    {
        initialize();
    }
    ~logger()
    {
        file.release();
    }
    void write(std::string _field, int _val)        //integer data
    {
        file << _field << _val;
    }
    void write(std::string _field, float _val)      //floating data
    {
        file << _field << _val;
    }
    void write(std::string _field, double _val)     //double data
    {
        file << _field << _val;
    }
    void write(std::string _field, cv::Mat &_val)   //Matrix data
    {
        file << _field << _val;
    }

    //void operator ()(std::string _field, cv::Mat &_val)
    //{
        //file << _field << _val;
    //}

//    template <datatype A>
    //void write(std::string _field, A &_val)
    //{
        //if(data_type(A) == data_tye(INT))
        //{
            //file << _field << _val;
        //}
        //else if(data_type(std::vector) == data_tpye())
        //{

        //}
        //else
        //{
            //printf("Un")
        //}

    //}
    void end()
    {
        printf("FILE[%s] is successfully closed!", file_name.c_str());
        file.release();
    }
};

#endif