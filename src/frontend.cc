#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

#include <opencv2/opencv.hpp>

namespace myslam
{

Frontend::Frontend()
{
    // nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold
    orb_ = cv::ORB::create(Config::Get<int>("num_features"), 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 20);
    // gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame)
{
    current_frame_ = frame;

    switch (status_)
    {
    case FrontendStatus::INITING:
        StereoInit();
        break;
    case FrontendStatus::TRACKING_GOOD:
    case FrontendStatus::TRACKING_BAD:
        Track();
        break;
    case FrontendStatus::LOST:
        Reset();
        break;
    }

    if (status_ == FrontendStatus::TRACKING_GOOD || status_ == FrontendStatus::TRACKING_BAD)
    {
        std::vector<double> tracked_frame{current_frame_->time_stamp_};
        std::vector<double> current_pose = Se3ToVectorD(current_frame_->Pose().inverse());  // Attention: we need to use T_w_c to recover the real camera pose
        tracked_frame.insert(tracked_frame.end(), current_pose.begin(), current_pose.end());
        all_tracked_frames_.emplace_back(tracked_frame);
        if (current_frame_->is_keyframe_)
            all_tracked_keyframes_.emplace_back(tracked_frame);
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track()
{
    if (last_frame_)
    {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_)
    {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
    }
    else if (tracking_inliers_ > num_features_tracking_bad_)
    {
        // tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    }
    else
    {
        // lost
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    if (viewer_)
        viewer_->AddCurrentFrame(current_frame_);

    return true;
}

bool Frontend::InsertKeyframe()
{
    if (tracking_inliers_ >= num_features_needed_for_keyframe_)
        return false; // still have enough features, don't insert keyframe

    // current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe " << current_frame_->keyframe_id_;

    SetObservationsForKeyFrame(); // add feature as observation of a map point
    DetectFeatures();             // detect new features
    FindFeaturesInRight();        // track in right image
    TriangulateNewPoints();       // triangulate map points
    backend_->UpdateMap();        // update backend (optimize) because we have a new keyframe

    if (viewer_)
        viewer_->UpdateMap();

    return true;
}

void Frontend::SetObservationsForKeyFrame()
{
    for (auto &feat : current_frame_->features_left_)
    {
        auto mp = feat->map_point_.lock();
        if (mp)
            mp->AddObservation(feat);
    }
}

int Frontend::TriangulateNewPoints()
{
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    SE3 current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    {
        // triangulate new map point only if
        // 1) no map point associated with this feature
        // 2) corresponding feature exists in right image
        if (current_frame_->features_left_[i]->map_point_.expired() &&
            current_frame_->features_right_[i] != nullptr)
        {
            std::vector<Vec3> points{
                camera_left_->Pixel2Camera(
                    Vec2(current_frame_->features_left_[i]->kp_.pt.x,
                         current_frame_->features_left_[i]->kp_.pt.y)),
                camera_right_->Pixel2Camera(
                    Vec2(current_frame_->features_right_[i]->kp_.pt.x,
                         current_frame_->features_right_[i]->kp_.pt.y))};
            Vec3 p_w = Vec3::Zero();

            // 1) the quality of the solution should be good
            // 2) the map point should be before the stereo camera
            if (triangulation(poses, points, p_w) && p_w[2] > 0)
            {
                auto new_map_point = MapPoint::CreateNewMappoint();
                p_w = current_pose_Twc * p_w; // be careful: we need to calculate the absolute position
                new_map_point->SetPos(p_w);
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);

                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                ++cnt_triangulated_pts;
            }
        }
    }
    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

int Frontend::EstimateCurrentPose()
{
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // set vertex pose
    VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    // camera intrinsics K
    Mat33 K = camera_left_->K();

    // set edges
    int index = 1;
    const double chi2_th = 5.991;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if (mp)
        {
            features.emplace_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(toVec2(current_frame_->features_left_[i]->kp_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(sqrt(chi2_th));
            edge->setRobustKernel(rk);

            edges.emplace_back(edge);
            optimizer.addEdge(edge);
            ++index;
        }
    }

    // estimate the Pose the determine the outliers
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration)
    {
        vertex_pose->setEstimate(current_frame_->Pose());

        // setLevel() is useful when you call optimizer.initializeOptimization().
        // If you assign initializeOptimization(0),
        // the optimizer will include all edges up to level 0 in the optimization,
        // and edges set to level >=1 will not be included.
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i)
        {
            auto e = edges[i];
            if (features[i]->is_outlier_)
            {
                e->computeError();
            }

            if (e->chi2() > chi2_th)
            {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                ++cnt_outlier;
            }
            else
            {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            }

            if (iteration == 2)
            {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/" << features.size() - cnt_outlier;

    // set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n"
              << current_frame_->Pose().matrix();

    for (auto &feat : features)
    {
        if (feat->is_outlier_)
        {
            feat->map_point_.reset();
            feat->is_outlier_ = false; // maybe we can still use it in future
        }
    }

    return features.size() - cnt_outlier;
}

int Frontend::TrackLastFrame()
{
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->features_left_)
    {
        if (kp->map_point_.lock())
        {
            // use project point
            auto mp = kp->map_point_.lock();
            auto px = camera_left_->World2Pixel(mp->pos_, current_frame_->Pose());
            kps_last.emplace_back(kp->kp_.pt);
            kps_current.emplace_back(cv::Point2f(px[0], px[1]));
        }
        else
        {
            kps_last.emplace_back(kp->kp_.pt);
            kps_current.emplace_back(kp->kp_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW); // use the input flow as an initial flow approximation

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i)
    {
        if (status[i])
        {
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.emplace_back(feature);
            ++num_good_pts;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}

bool Frontend::StereoInit()
{
    /* int num_features_left = */ DetectFeatures();
    int num_coor_features = FindFeaturesInRight();
    if (num_coor_features < num_features_init_)
        return false;

    bool build_map_success = BuildInitMap();
    if (build_map_success)
    {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_)
        {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

int Frontend::DetectFeatures()
{
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255); // ROI: region of interest
    for (auto &feat : current_frame_->features_left_)
    {
        cv::rectangle(mask,
                      feat->kp_.pt - cv::Point2f(10, 10),
                      feat->kp_.pt + cv::Point2f(10, 10),
                      0,
                      CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    orb_->detect(current_frame_->left_img_, keypoints, mask);
    // gftt_->detect(current_frame_->left_img_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints)
    {
        current_frame_->features_left_.emplace_back(Feature::Ptr(new Feature(current_frame_, kp)));
        ++cnt_detected;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

int Frontend::FindFeaturesInRight()
{
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame_->features_left_)
    {
        kps_left.emplace_back(kp->kp_.pt);
        auto mp = kp->map_point_.lock();
        if (mp)
        {
            // use projected points as initial guess
            auto px = camera_right_->World2Pixel(mp->pos_, current_frame_->Pose());
            kps_right.emplace_back(cv::Point2f(px[0], px[1]));
        }
        else
        {
            // use same pixel in left iamge
            kps_right.emplace_back(kp->kp_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW); // use the input flow as an initial flow approximation

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i)
    {
        if (status[i])
        {
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.emplace_back(feat);
            ++num_good_pts;
        }
        else
        {
            current_frame_->features_right_.emplace_back(nullptr);
        }
    }
    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
    return num_good_pts;
}

bool Frontend::BuildInitMap()
{
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;

    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    {
        if (current_frame_->features_right_[i] == nullptr)
            continue;

        // create map point from triangulation
        std::vector<Vec3> points{
            camera_left_->Pixel2Camera(
                Vec2(current_frame_->features_left_[i]->kp_.pt.x,
                     current_frame_->features_left_[i]->kp_.pt.y)),
            camera_right_->Pixel2Camera(
                Vec2(current_frame_->features_right_[i]->kp_.pt.x,
                     current_frame_->features_right_[i]->kp_.pt.y))};
        Vec3 p_w = Vec3::Zero();

        // 1) the quality of the solution should be good
        // 2) the map point should be before the stereo camera
        if (triangulation(poses, points, p_w) && p_w[2] > 0)
        {
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(p_w);
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);
            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;
            ++cnt_init_landmarks;
            map_->InsertMapPoint(new_map_point);
        }
    }
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks << " map points";

    return true;
}

bool Frontend::Reset()
{
    LOG(INFO) << "Reset is not implemented.";
    return true;
}

} // namespace myslam