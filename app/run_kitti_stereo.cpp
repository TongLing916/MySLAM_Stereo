#include "myslam/visual_odometry.h"

#include <gflags/gflags.h>

DEFINE_string(config_file, "./config/default.yaml", "config file path");

int main(int argc, char **argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
