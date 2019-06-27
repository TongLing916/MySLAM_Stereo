#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace myslam
{

/** 
* Configuration Reader
* Use SetParameterFile to define the configuration file.
* Then, use Get to obtain the value.
* Singleton  
*/
class Config
{
public:
    // close the file when deconstructing
    ~Config();

    // set a new config file
    static bool SetParameterFile(const std::string &filename);

    // access the parameter values
    template <typename T>
    static T Get(const std::string &key)
    {
        return T(Config::config_->file_[key]);
    }

private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    // private constructor makes a singleton
    Config()
    {
    }
};
} // namespace myslam

#endif // MYSLAM_CONFIG_H
