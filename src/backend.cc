#include "myslam/backend.h"
#include "myslam/algorithm.h"
#include "myslam/feature.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"

namespace myslam
{

Backend::Backend()
{
    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
}

void Backend::UpdateMap()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void Backend::Stop()
{
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}

void Backend::BackendLoop()
{
    while (backend_running_.load())
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);

        // only optimize the activated keyframesa and landmarks
        Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
        Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
        Optimize(active_kfs, active_landmarks);
    }
}

void Backend::Optimize(Map::KeyframesType &keyframes,
                       Map::LandmarksType &landmarks)
{
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType; // 6: Dimension of camera pose, 3: Dimension of landmark's position
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // init maps to manage vertices and edges
    std::map<unsigned long, VertexPose *> vertices_poses;        // <keyframe id, vertex keyframe pose>
    std::map<unsigned long, VertexXYZ *> vertices_landmarks;     // <map point id, vertex landmark position>
    std::map<EdgeProjection *, Feature::Ptr> edges_and_features; // <edge projection, shared pointer feature>

    // set vertex "camera pose" using keyframe id as index
    unsigned long max_kf_id = 0;
    for (auto &keyframe : keyframes)
    {
        auto kf = keyframe.second;
        VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->Pose());
        optimizer.addVertex(vertex_pose);
        if (kf->keyframe_id_ > max_kf_id)
        {
            max_kf_id = kf->keyframe_id_;
        }

        vertices_poses.insert({kf->keyframe_id_, vertex_pose});
    }

    // get intrinsics K and left / right extrinsics
    Mat33 K = cam_left_->K();
    SE3 left_ext = cam_left_->pose();
    SE3 right_ext = cam_right_->pose();

    // threshold for robust kernel --> Probability 95%
    // ref: <<MVG>> Appendix 2.2 chi square
    double chi2_th = 5.991;

    // set edge "projection"
    int index = 1; // edge index
    for (auto &landmark : landmarks)
    {
        if (landmark.second->is_outlier_)
            continue;
        unsigned long landmark_id = landmark.second->id_;
        auto observations = landmark.second->GetObs();
        for (auto &obs : observations)
        {
            // check the validity of the observed feature.
            // if expired / removed, lock() returns nullptr
            if (obs.lock() == nullptr)
                continue;

            auto feat = obs.lock();
            if (feat->is_outlier_ || feat->frame_.lock() == nullptr)
                continue;

            auto frame = feat->frame_.lock();

            EdgeProjection *edge = nullptr;
            if (feat->is_on_left_image_)
            {
                edge = new EdgeProjection(K, left_ext);
            }
            else
            {
                edge = new EdgeProjection(K, right_ext);
            }

            // if the landmark is not yet added into optimization, add a new vertex landmark
            if (vertices_landmarks.find(landmark_id) ==
                vertices_landmarks.end())
            {
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second->Pos());
                v->setId(landmark_id + max_kf_id + 1);
                v->setMarginalized(true); // use the sparsity in information matrix to accelrate the optimization (see chapter 10 in <<Visual SLAM 14 lectures>>)
                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v);
            }

            edge->setId(index);
            edge->setVertex(0, vertices_poses.at(frame->keyframe_id_)); // pose
            edge->setVertex(1, vertices_landmarks.at(landmark_id));     // landmark
            edge->setMeasurement(toVec2(feat->kp_.pt));
            edge->setInformation(Mat22::Identity());
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(sqrt(chi2_th));
            edge->setRobustKernel(rk);
            edges_and_features.insert({edge, feat});

            optimizer.addEdge(edge);

            index++;
        }
    }

    // do optimization and eliminate the outliers
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;
    while (iteration < 5)
    {
        cnt_outlier = 0;
        cnt_inlier = 0;
        // determine if we want to adjust the outlier threshold
        for (auto &ef : edges_and_features)
        {
            if (ef.first->chi2() > chi2_th)
                ++cnt_outlier;
            else
                ++cnt_inlier;
        }
        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if (inlier_ratio > 0.5)
        {
            break;
        }
        else
        {
            chi2_th *= 2;
            ++iteration;
        }
    }

    for (auto &ef : edges_and_features)
    {
        if (ef.first->chi2() > chi2_th)
        {
            ef.second->is_outlier_ = true;
            // remove the observation
            ef.second->map_point_.lock()->RemoveObservation(ef.second);
        }
        else
        {
            ef.second->is_outlier_ = false;
        }
    }

    LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/" << cnt_inlier;

    // Set pose and lanrmark position
    for (auto &v : vertices_poses)
        keyframes.at(v.first)->SetPose(v.second->estimate());
    for (auto &v : vertices_landmarks)
        landmarks.at(v.first)->SetPos(v.second->estimate());
}

} // namespace myslam