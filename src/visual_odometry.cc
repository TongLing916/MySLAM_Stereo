#include "myslam/visual_odometry.h"
#include "myslam/config.h"

#include <chrono>

namespace myslam
{

VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path)
{
}

bool VisualOdometry::Init()
{
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false)
        return false;

    dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->Init(), true);

    // create components and links
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);

    return true;
}

void VisualOdometry::Run()
{
    while (1)
    {
        LOG(INFO) << "VO is running";
        if (Step() == false)
        {
            break;
        }
    }

    backend_->Stop();
    viewer_->Close();

    PrintAllKeyFrames();
    PrintAllFrames();

    LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step()
{
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr)
        return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}

void VisualOdometry::PrintAllFrames()
{
    std::string prefix = "AllFrames_";

    // obtain the current time
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    auto time_str = oss.str();

    std::string output_name = prefix + time_str + ".txt";

    std::vector<std::vector<double>> all_tracked_frames = frontend_->GetAllTrackedFrames();
    if (all_tracked_frames.size() > 0)
    {
        std::ofstream f;
        f.open(output_name);

        for (const auto &tf : all_tracked_frames)
        {
            f << std::setprecision(5) << tf[0] << " "
              << tf[1] << " " << tf[2] << " " << tf[3] << " " <<
                tf[4] << " " << tf[5] << " " << tf[6] << " " << tf[7] << std::endl;
        }

        f.close();
    }
    else
    {
        LOG(WARNING) << "No frame was tracked :( Good luck next time!";
    }
}

void VisualOdometry::PrintAllKeyFrames()
{
    std::string prefix = "AllKeyFrames_";

    // obtain the current time
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    auto time_str = oss.str();

    std::string output_name = prefix + time_str + ".txt";

    std::vector<std::vector<double>> all_tracked_keyframes = frontend_->GetAllTrackedKeyFrames();
    if (all_tracked_keyframes.size() > 0)
    {
        std::ofstream f;
        f.open(output_name);

        for (const auto &tf : all_tracked_keyframes)
        {
            f << std::setprecision(5) << tf[0] << " "
              << tf[1] << " " << tf[2] << " " << tf[3] << " " <<
                tf[4] << " " << tf[5] << " " << tf[6] << " " << tf[7] << std::endl;
        }

        f.close();
    }
    else
    {
        LOG(WARNING) << "No frame was tracked :( Good luck next time!";
    }
}

} // namespace myslam
