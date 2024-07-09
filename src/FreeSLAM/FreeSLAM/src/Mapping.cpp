#include "FreeSLAM/Mapping.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <pcl/common/io.h>

#include <chrono>
#include <fstream>

#include "FreeSLAM/Deskew.hpp"
#include "FreeSLAM/Downsample.hpp"
#include "FreeSLAM/Registration.hpp"


struct VelocityData {
    Eigen::Matrix<float, 6, 1> velocity;
    double time_diff;
};



namespace FreeSLAM {

std::vector<VelocityData> velocity_data_vec;
auto last_save_time = std::chrono::steady_clock::now();
Eigen::Matrix<float, 6, 1> prev_velocity = Eigen::Matrix<float, 6, 1>::Zero();

Mapping::Mapping(const MappingConfigs &configs)
    : MappingConfigs(configs), p_voxel(std::make_shared<HashVoxel>(voxel_cfg)) {
    if (enable_loopclosure) {
        loop_running = true;
        loop_thread = std::thread(&Mapping::LoopClosure, this);
    }

    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 1;
    p_isam2 = std::make_unique<gtsam::ISAM2>(params);

    mapping_vec.reserve(1000000);
    tracking_vec.reserve(1000000);
    pose_vec.reserve(1000000);
    stamp_vec.reserve(1000000);
}

Mapping::~Mapping() { Shutdown(); }

void Mapping::Shutdown() {
    loop_running = false;
    if (loop_thread.joinable()) loop_thread.join();
    

    using namespace gtsam::symbol_shorthand;
    gtsam::Values result = p_isam2->calculateEstimate();
    int frame_num = pose_vec.size();
    for (int i = 0; i < frame_num; i++) {
        pose_vec[i] = result.at<gtsam::Pose3>(P(i)).matrix().cast<float>();
    }
}

void Mapping::checkAndSaveVelocityData() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_save_time).count();
    if (duration >= 60) { 
        saveVelocityData(velocity_data_vec, "velocity_data.csv");
        last_save_time = now;
    }
}

void Mapping::FeedPointCloud(double stamp, PointVec frame) {
    std::vector<int> frame_indices;
    // filter points by range
    RangeSample(frame, frame_indices, range_min, range_max);
    // downsample points
    std::vector<int> tracking_indices = frame_indices;
    RandomVoxelSample(frame, tracking_indices, tracking_voxel);
    // deskew frame by constant velocity assumption
    if (enable_deskew) {
        double stamp_max =
            std::max_element(frame.begin(), frame.end(), [](const auto &p0, const auto &p1) {
                return p0.timestamp < p1.timestamp;
            })->timestamp;
        Eigen::Vector6f deskew_delta = velocity * (stamp_max - stamp);
        Deskew(frame, stamp, stamp_max, frame_indices, deskew_delta, false);
        stamp = stamp_max;
    }
    // apply extinct
    if (extinct.size() == 3) {
        for (auto &pt : frame) {
            pt.getVector3fMap() -= Eigen::Map<const Eigen::Vector3f>(extinct.data());
        }
    }
    // copy input shared data which need lock
    std::unique_lock lock(mtx);
    int frame_num = pose_vec.size();
    Eigen::Matrix4f prev_pose;
    
    Eigen::Matrix4f predicted = Eigen::Matrix4f::Identity();
    std::shared_ptr<HashVoxel> p_voxel_locked;
    {
        // std::unique_lock lock(mtx);
        p_voxel_locked = p_voxel;
        if (frame_num >= 1) {
            prev_pose = pose_vec.back();
            predicted = prev_pose * SE3Exp(velocity * (stamp - stamp_vec.back()));
        }
    }
    // frame to map registration
    Eigen::Matrix4f curr_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f curr_delta = Eigen::Matrix4f::Identity();
    Eigen::Matrix6f information = Eigen::Matrix6f::Identity() * 1e8;
    if (frame_num >= static_frames) {
        Eigen::Matrix4f initial = predicted;
        auto result =
            Registration(*p_voxel_locked, frame, tracking_indices, initial, reg_cfg_track);
        double overlap = result.matches / (double)tracking_indices.size();
        std::cout << "[Track]: matches = [" << result.matches << "/" << tracking_indices.size()
                  << "]"
                  << ", overlap = " << overlap << ", iterations = " << result.iterations
                  << ", fitness = " << result.fitness << std::endl;
        curr_pose = initial * result.delta;
        curr_delta = prev_pose.inverse() * curr_pose;
        information = result.information_matrix;
        velocity = SE3Log(curr_delta) / (stamp - stamp_vec.back());

        Eigen::Matrix<float, 6, 1> threshold;
        threshold << 0.1, 0.1, 0.1, 1.0, 3.0, 1.0; // 分别对应每个分量的阈值

        Eigen::Matrix<float, 6, 1> diff = (velocity - prev_velocity).cwiseAbs();
        if ((diff.array() < threshold.array()).all()) {
            // std::cout << "ok"<< std::endl;
            prev_velocity = velocity;
        } else {
            std::cout << "not ok"<< std::endl;
            // std::cout << stamp_vec.back() << std::endl;
            // std::cout << pose_vec.back() << std::endl;
            std::cout <<diff<< std::endl;
            velocity = prev_velocity;
            // return;
        }
        // VelocityData data;
        // data.velocity = velocity;
        // data.time_diff = stamp - stamp_vec.back();
        // velocity_data_vec.push_back(data);
        // checkAndSaveVelocityData();
        // std::cout << "velocity"<< velocity<<std::endl;
    }
    // prepare mapping indices
    std::vector<int> mapping_indices;
    if (tracking_voxel != mapping_voxel) {
        mapping_indices = frame_indices;
        RandomVoxelSample(frame, mapping_indices, mapping_voxel);
    } else {
        mapping_indices = tracking_indices;
    }
    // prepare mapping frame
    PointVec mapping_frame;
    pcl::copyPointCloud(frame, mapping_indices, mapping_frame);
    // prepare tracking frame
    PointVec tracking_frame;
    pcl::copyPointCloud(frame, tracking_indices, tracking_frame);
    // prepare update isam2
    using namespace gtsam::symbol_shorthand;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    if (frame_num == 0) {
        auto noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << M_PI, M_PI, M_PI, 1e2, 1e2, 1e2).finished());
        // auto noise = gtsam::noiseModel::Constrained::Sigmas(
        //     (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
        gtsam::Pose3 pose3(curr_pose.cast<double>());
        gtsam::PriorFactor<gtsam::Pose3> factor(P(0), pose3, noise);
        graph.add(factor);
        initialEstimate.insert(P(0), pose3);
    } else {
        auto noise = gtsam::noiseModel::Gaussian::Information(information.cast<double>());
        // auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        //     (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());
        gtsam::Pose3 between(curr_delta.cast<double>());
        gtsam::BetweenFactor<gtsam::Pose3> factor(P(frame_num - 1), P(frame_num), between, noise);
        gtsam::Pose3 pose3(curr_pose.cast<double>());
        graph.add(factor);
        initialEstimate.insert(P(frame_num), pose3);
    }
    // save output shared data which need lock
    {
        // std::unique_lock lock(mtx);
        // update isam2
        p_isam2->update(graph, initialEstimate);
        // update curr_pose
        if (!pose_vec.empty()) curr_pose = pose_vec.back() * curr_delta;
        // save results
        stamp_vec.emplace_back(stamp);
        pose_vec.emplace_back(curr_pose);
        tracking_vec.emplace_back(tracking_frame);
        mapping_vec.emplace_back(mapping_frame);

        p_voxel_locked->AddPoints(mapping_frame, {}, curr_pose);
        // std::cout<< "stamp_vec:" << stamp_vec.capacity() * sizeof(stamp_vec[0]) / (1024.0 * 1024.0) << "MB" <<std::endl;
        // std::cout<< "pose_vec:" << pose_vec.capacity() * sizeof(pose_vec[0]) / (1024.0 * 1024.0) << "MB" <<std::endl;
        // std::cout<< "tracking_vec:" << tracking_vec.capacity() * sizeof(tracking_vec[0]) / (1024.0 * 1024.0) << "MB" <<std::endl;
        // std::cout<< "mapping_vec:" << mapping_vec.capacity() * sizeof(mapping_vec[0]) / (1024.0 * 1024.0) << "MB" <<std::endl;
        // std::cout<< "p_voxel_locked:" << p_voxel_locked.capacity() * sizeof(p_voxel_locked[0]) / (1024.0 * 1024.0) << "MB" <<std::endl;
    }
}

void Mapping::FeedGPS(double stamp, const Eigen::Vector3d &utm) {
    if (pose_vec.size() < 100) return;
    if (p_utm_origin == nullptr) {
        p_utm_origin = std::make_unique<Eigen::Vector3d>(utm);
    }
    Eigen::Vector3d local_utm = utm - *p_utm_origin;
    gps_map[stamp] = local_utm;

    if (gps_map.size() < 2) return;
    auto it = gps_map.end();
    --it;
    --it;

    std::unique_lock lock(mtx);

    auto upper_it = std::upper_bound(stamp_vec.begin(), stamp_vec.end(), it->first);
    auto lower_it = upper_it;
    lower_it--;

    auto closest_it = (*upper_it - it->first < it->first - *lower_it) ? upper_it : lower_it;
    int closest_idx = closest_it - stamp_vec.begin();

    if (!((*upper_it >= it->first) && (*lower_it <= it->first))) {
        exit(-1);
    }

    using namespace gtsam::symbol_shorthand;
    gtsam::NonlinearFactorGraph graph;
    auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1, 1, 1).finished());
    gtsam::GPSFactor factor(P(closest_idx), it->second, noise);
    graph.add(factor);

    // save output shared data which need lock
    int frame_num;
    {
        //
        frame_num = pose_vec.size();
        // update isam2
        p_isam2->update(graph);
        for (int i = 0; i < 2; i++) p_isam2->update();
        // save poses
        gtsam::Values result = p_isam2->calculateEstimate();
        for (int i = 0; i < frame_num; i++) {
            pose_vec[i] = result.at<gtsam::Pose3>(P(i)).matrix().cast<float>();
        }
        if (update_local_map) {
            // build new voxel map
            auto p_new_voxel = std::make_shared<HashVoxel>(voxel_cfg);
            for (int i = frame_num - 1; i >= std::max(0, frame_num - loop_local_frames); i--) {
                p_new_voxel->AddPoints(mapping_vec[i], {}, pose_vec[i]);
                if ((int)p_new_voxel->GetVoxelNum() >= voxel_cfg.max_num_voxels) break;
            }
            // update new voxel map
            p_voxel = p_new_voxel;
        }
    }
}

void Mapping::LoopClosure() {
    std::cout << "[Loop]: loop-closure enable." << std::endl;
    std::random_device rd;
    std::default_random_engine rng(rd());
    std::shared_ptr<HashVoxel> p_loop_voxel;
    while (loop_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(loop_interval_ms));
        if ((int)pose_vec.size() < loop_frame_skip) continue;

        std::unique_lock lock(mtx);

        int current_id = pose_vec.size() - loop_frame_skip / 2;
        Eigen::Matrix4f current_pose = pose_vec[current_id];

        std::vector<int> candidates;
        for (int i = 2; i < (int)pose_vec.size() - loop_frame_skip; i++) {
            float distance = (pose_vec[i].inverse() * current_pose).topRightCorner<3, 1>().norm();
            if (distance < loop_distance_threshold) candidates.emplace_back(i);
        }
        if (candidates.empty()) continue;
        std::shuffle(candidates.begin(), candidates.end(), rng);

        {
            // std::unique_lock lock(mtx);
            p_loop_voxel = std::make_shared<HashVoxel>(*p_voxel);
        }
        int trials = std::min((int)candidates.size(), loop_trials_num);
        int history_id = -1;
        Eigen::Matrix4f closed_pose;
        Eigen::Matrix6f information;
        PointVec *p_history_frame = nullptr;
        bool ok = false;
        for (int i = 0; i < trials; i++) {
            history_id = candidates[i];
            const auto &history_pose = pose_vec[history_id];
            p_history_frame = tracking_vec[history_id].size() > mapping_vec[history_id].size()
                                  ? &tracking_vec[history_id]
                                  : &mapping_vec[history_id];
            auto result =
                Registration(*p_loop_voxel, *p_history_frame, {}, history_pose, reg_cfg_loop);
            double overlap = result.matches / (double)p_history_frame->size();
            std::cout << "[Loop]: matches = [" << result.matches << "/" << p_history_frame->size()
                      << "]"
                      << ", overlap = " << overlap << ", iterations = " << result.iterations
                      << ", fitness = " << result.fitness << std::endl;
            std::cout << "[Loop]: delta = " << std::endl << result.delta << std::endl;
            if (result.fitness > loop_fitness_threshold) continue;
            if (overlap < loop_overlap_threshold) continue;
            closed_pose = history_pose * result.delta;
            information = result.information_matrix;
            ok = true;
            break;
        }
        if (!ok) continue;

        std::cout << "[Loop]: loop found." << std::endl;

        using namespace gtsam::symbol_shorthand;
        gtsam::NonlinearFactorGraph graph;
        auto noise = gtsam::noiseModel::Gaussian::Information(information.cast<double>());
        // auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        //     (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());
        gtsam::Pose3 between((closed_pose.inverse() * current_pose).cast<double>());
        gtsam::BetweenFactor<gtsam::Pose3> factor(P(history_id), P(current_id), between, noise);
        graph.add(factor);

        // save output shared data which need lock
        int frame_num;
        {
            // std::unique_lock lock(mtx);
            //
            frame_num = pose_vec.size();
            // update isam2
            p_isam2->update(graph);
            for (int i = 0; i < 10; i++) p_isam2->update();
            // save poses
            gtsam::Values result = p_isam2->calculateEstimate();
            for (int i = 0; i < frame_num; i++) {
                pose_vec[i] = result.at<gtsam::Pose3>(P(i)).matrix().cast<float>();
            }

            if (update_local_map) {
                // build new voxel map
                p_loop_voxel->Reset();
                for (int i = frame_num - 1; i >= std::max(0, frame_num - loop_local_frames); i--) {
                    p_loop_voxel->AddPoints(mapping_vec[i], {}, pose_vec[i]);
                    if ((int)p_loop_voxel->GetVoxelNum() >= voxel_cfg.max_num_voxels) break;
                }
                // update new voxel map
                p_voxel = p_loop_voxel;
            }

            // save loop frame points
            loop_frame = *p_history_frame;
            for (auto &pt : loop_frame) {
                pt.getVector4fMap() = closed_pose * pt.getVector4fMap();
            }
        }
    }
}

void Mapping::saveVelocityData(const std::vector<VelocityData>& data_vec, const std::string& filename) const{
    std::ofstream file(filename);
    if (file.is_open()) {
        file << "velocity_x,velocity_y,velocity_z,velocity_roll,velocity_pitch,velocity_yaw,time_diff\n";
        for (const auto& data : data_vec) {
            for (int i = 0; i < 6; ++i) {
                file << data.velocity(i) << ",";
            }
            file << data.time_diff << "\n";
        }
        file.close();
    }
}

void Mapping::GetGlobalMapPoints(PointVec &points) const {
    points.clear();
    // std::unique_lock lock(mtx);
    PointVec tmp;
    int frame_num = pose_vec.size();
    for (int i = 0; i < frame_num; i++) {
        tmp = mapping_vec[i];
        for (auto &pt : tmp) {
            pt.getVector4fMap() = pose_vec[i] * pt.getVector4fMap();
        }
        points += tmp;
    }
}

}  // namespace FreeSLAM