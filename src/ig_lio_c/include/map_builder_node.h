#ifndef _MAP_BUILDER_NODE_H
#define _MAP_BUILDER_NODE_H

#include <map>
#include <mutex>
#include <vector>
#include <thread>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <boost/optional.hpp>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>
#include <filters/filter_chain.hpp>

#include <ig_lio_c_msgs/srv/save_map.hpp>
#include <ig_lio_c_msgs/srv/re_loc.hpp>

#include "./ig_lio_c/map_builder/iglio_builder.h"
#include "./ig_lio_c/localizer/icp_localizer.h"

namespace IG_LIO
{
    using namespace std::chrono;
    using std::placeholders::_1;
    using std::placeholders::_2;
    namespace gm = ::grid_map::grid_map_pcl;

    struct LoopPair
    {
        LoopPair(int p, int c, float s, Eigen::Matrix3d &dr, Eigen::Vector3d &dp) : pre_idx(p), cur_idx(c), score(s), diff_rot(dr), diff_pos(dp) {}
        int pre_idx;
        int cur_idx;
        Eigen::Matrix3d diff_rot;
        Eigen::Vector3d diff_pos;
        double score;
    };

    struct Pose6D
    {
        Pose6D(int i, double t, Eigen::Matrix3d lr, Eigen::Vector3d lp) : index(i), time(t), local_rot(lr), local_pos(lp) {}
        void setGlobalPose(const Eigen::Matrix3d &gr, const Eigen::Vector3d &gp)
        {
            global_rot = gr;
            global_pos = gp;
        }
        void addOffset(const Eigen::Matrix3d &offset_rot, const Eigen::Vector3d &offset_pos)
        {
            global_rot = offset_rot * local_rot;
            global_pos = offset_rot * local_pos + offset_pos;
        }

        void getOffset(Eigen::Matrix3d &offset_rot, Eigen::Vector3d &offset_pos)
        {
            offset_rot = global_rot * local_rot.transpose();
            offset_pos = -global_rot * local_rot.transpose() * local_pos + global_pos;
        }
        int index;
        double time;
        Eigen::Matrix3d local_rot;
        Eigen::Vector3d local_pos;
        Eigen::Matrix3d global_rot;
        Eigen::Vector3d global_pos;
    };

    struct SharedData
    {
        bool key_pose_added = false;
        bool pose_updated = false;
        bool localizer_activate = false;
        bool localizer_service_called = false;
        bool localizer_service_success = false;
        std::mutex service_mutex;
        std::mutex mutex;
        Eigen::Matrix3d offset_rot = Eigen::Matrix3d::Identity();
        Eigen::Vector3d offset_pos = Eigen::Vector3d::Zero();
        std::vector<Pose6D> key_poses;
        std::vector<LoopPair> loop_pairs;
        std::vector<std::pair<int, int>> loop_history;
        std::vector<IG_LIO::PointCloudXYZI::Ptr> cloud_history;
        std::string map_path;
        Eigen::Matrix3d local_rot;
        Eigen::Vector3d local_pos;
        Eigen::Matrix4d initial_guess;
        IG_LIO::PointCloudXYZI::Ptr cloud;
        bool reset_flag = false;
        bool halt_flag = false;
    };

    struct LoopParams
    {
        double rad_thresh = 0.4;
        double dist_thresh = 2.5;
        double time_thresh = 30.0;
        double loop_pose_search_radius = 10.0;
        int loop_pose_index_thresh = 5;
        double submap_resolution = 0.2;
        int submap_search_num = 20;
        double loop_icp_thresh = 0.3;
        bool activate = false;
    };

    class LoopClosureThread
    {
    public:
        void init()
        {

            gtsam::ISAM2Params isam2_params;
            isam2_params.relinearizeThreshold = 0.01;
            isam2_params.relinearizeSkip = 1;
            isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);
            kdtree_history_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
            cloud_history_poses_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            sub_map_downsize_filter_.reset(new pcl::VoxelGrid<IG_LIO::PointType>);
            sub_map_downsize_filter_->setLeafSize(loop_params_.submap_resolution,
                                                  loop_params_.submap_resolution,
                                                  loop_params_.submap_resolution);

            icp_.reset(new pcl::IterativeClosestPoint<IG_LIO::PointType, IG_LIO::PointType>);
            icp_->setMaxCorrespondenceDistance(100);
            icp_->setMaximumIterations(50);
            icp_->setTransformationEpsilon(1e-6);
            icp_->setEuclideanFitnessEpsilon(1e-6);
            icp_->setRANSACIterations(0);
        }
        void setShared(std::shared_ptr<SharedData> share_data)
        {
            shared_data_ = share_data;
        }
        void setRate(const double &rate)
        {
            rate_ = std::make_shared<rclcpp::Rate>(rate);
        }
        void setRate(std::shared_ptr<rclcpp::Rate> rate)
        {
            rate_ = rate;
        }
        LoopParams &mutableParams()
        {
            return loop_params_;
        }

        IG_LIO::PointCloudXYZI::Ptr getSubMaps(std::vector<Pose6D> &pose_list,
                                               std::vector<IG_LIO::PointCloudXYZI::Ptr> &cloud_list,
                                               int index,
                                               int search_num)
        {
            IG_LIO::PointCloudXYZI::Ptr cloud(new IG_LIO::PointCloudXYZI);
            int max_size = pose_list.size();
            int min_index = std::max(0, index - search_num);
            int max_index = std::min(max_size - 1, index + search_num);
            for (int i = min_index; i <= max_index; i++)
            {
                Pose6D &p = pose_list[i];
                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                T.block<3, 3>(0, 0) = p.global_rot;
                T.block<3, 1>(0, 3) = p.global_pos;
                IG_LIO::PointCloudXYZI::Ptr temp_cloud(new IG_LIO::PointCloudXYZI);
                pcl::transformPointCloud(*cloud_list[p.index], *temp_cloud, T);
                *cloud += *temp_cloud;
            }
            sub_map_downsize_filter_->setInputCloud(cloud);
            sub_map_downsize_filter_->filter(*cloud);
            return cloud;
        }

        void operator()()
        {
            while (is_alive)
            {
                rate_->sleep();
                if (terminate_flag)
                {
                    is_alive = !terminate_flag;
                    break;
                }
                if (!loop_params_.activate)
                    continue;
                if (shared_data_->key_poses.size() < loop_params_.loop_pose_index_thresh)
                    continue;
                if (!shared_data_->key_pose_added)
                    continue;
                shared_data_->key_pose_added = false;
                {
                    std::lock_guard<std::mutex> lock(shared_data_->mutex);
                    lastest_index_ = shared_data_->key_poses.size() - 1;
                    temp_poses_.clear();
                    temp_poses_.assign(shared_data_->key_poses.begin(), shared_data_->key_poses.end());
                }
                loopCheck();
                addOdomFactor();
                addLoopFactor();
                smoothAndUpdate();
            }
        }

        void stop()
        {
            terminate_flag = true;
        }

    private:
        std::shared_ptr<SharedData> shared_data_;

        std::shared_ptr<rclcpp::Rate> rate_;

        LoopParams loop_params_;

        std::vector<Pose6D> temp_poses_;

        int previous_index_ = 0;

        int lastest_index_;

        bool loop_found_ = false;

        bool is_alive = true;

        bool terminate_flag = false;

        gtsam::Values initialized_estimate_;

        gtsam::Values optimized_estimate_;

        std::shared_ptr<gtsam::ISAM2> isam2_;

        gtsam::NonlinearFactorGraph gtsam_graph_;

        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_history_poses_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_history_poses_;

        pcl::VoxelGrid<IG_LIO::PointType>::Ptr sub_map_downsize_filter_;

        pcl::IterativeClosestPoint<IG_LIO::PointType, IG_LIO::PointType>::Ptr icp_;

        void loopCheck()
        {
            if (temp_poses_.empty())
                return;
            int cur_index = temp_poses_.size() - 1;
            int pre_index = -1;

            cloud_history_poses_->clear();

            for (Pose6D &p : temp_poses_)
            {
                pcl::PointXYZ point;
                point.x = p.global_pos(0);
                point.y = p.global_pos(1);
                point.z = p.global_pos(2);
                cloud_history_poses_->push_back(point);
            }
            kdtree_history_poses_->setInputCloud(cloud_history_poses_);
            std::vector<int> ids;
            std::vector<float> sqdists;
            kdtree_history_poses_->radiusSearch(cloud_history_poses_->back(), loop_params_.loop_pose_search_radius, ids, sqdists, 0);

            for (int i = 0; i < ids.size(); i++)
            {
                int id = ids[i];
                if (std::abs(temp_poses_[id].time - temp_poses_.back().time) > loop_params_.time_thresh && std::abs(cur_index - id) >= loop_params_.loop_pose_index_thresh)
                {
                    pre_index = id;
                    break;
                }
            }
            if (pre_index == -1 || pre_index == cur_index)
                return;

            IG_LIO::PointCloudXYZI::Ptr cur_cloud = getSubMaps(temp_poses_, shared_data_->cloud_history, cur_index, 0);
            IG_LIO::PointCloudXYZI::Ptr sub_maps = getSubMaps(temp_poses_, shared_data_->cloud_history, pre_index, loop_params_.submap_search_num);

            icp_->setInputSource(cur_cloud);
            icp_->setInputTarget(sub_maps);

            IG_LIO::PointCloudXYZI::Ptr aligned(new IG_LIO::PointCloudXYZI);

            icp_->align(*aligned, Eigen::Matrix4f::Identity());

            float score = icp_->getFitnessScore();

            if (!icp_->hasConverged() || score > loop_params_.loop_icp_thresh)
                return;

            std::cout << "Detected LOOP: " << pre_index << " " << cur_index << " " << score << std::endl;
            shared_data_->loop_history.emplace_back(pre_index, cur_index);
            loop_found_ = true;

            Eigen::Matrix4d T_pre_cur = icp_->getFinalTransformation().cast<double>();
            Eigen::Matrix3d R12 = temp_poses_[pre_index].global_rot.transpose() * T_pre_cur.block<3, 3>(0, 0) * temp_poses_[cur_index].global_rot;
            Eigen::Vector3d t12 = temp_poses_[pre_index].global_rot.transpose() * (T_pre_cur.block<3, 3>(0, 0) * temp_poses_[cur_index].global_pos + T_pre_cur.block<3, 1>(0, 3) - temp_poses_[pre_index].global_pos);
            shared_data_->loop_pairs.emplace_back(pre_index, cur_index, score, R12, t12);
        }

        void addOdomFactor()
        {
            for (int i = previous_index_; i < lastest_index_; i++)
            {
                Pose6D &p1 = temp_poses_[i];
                Pose6D &p2 = temp_poses_[i + 1];

                if (i == 0)
                {
                    initialized_estimate_.insert(i, gtsam::Pose3(gtsam::Rot3(p1.local_rot),
                                                                 gtsam::Point3(p1.local_pos)));
                    gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
                    gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(i, gtsam::Pose3(gtsam::Rot3(p1.local_rot), gtsam::Point3(p1.local_pos)), noise));
                }
                initialized_estimate_.insert(i + 1, gtsam::Pose3(gtsam::Rot3(p2.local_rot),
                                                                 gtsam::Point3(p2.local_pos)));
                Eigen::Matrix3d R12 = p1.local_rot.transpose() * p2.local_rot;
                Eigen::Vector3d t12 = p1.local_rot.transpose() * (p2.local_pos - p1.local_pos);

                gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
                gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(i,
                                                                    i + 1,
                                                                    gtsam::Pose3(gtsam::Rot3(R12), gtsam::Point3(t12)), noise));
            }
            previous_index_ = lastest_index_;
        }

        void addLoopFactor()
        {
            if (!loop_found_)
                return;
            if (shared_data_->loop_pairs.empty())
                return;
            for (LoopPair &lp : shared_data_->loop_pairs)
            {
                gtsam::Pose3 pose_between(gtsam::Rot3(lp.diff_rot), gtsam::Point3(lp.diff_pos));
                gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(lp.pre_idx,
                                                                    lp.cur_idx,
                                                                    pose_between,
                                                                    gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * lp.score)));
            }
            shared_data_->loop_pairs.clear();
        }

        void smoothAndUpdate()
        {
            isam2_->update(gtsam_graph_, initialized_estimate_);
            isam2_->update();
            if (loop_found_)
            {
                isam2_->update();
                isam2_->update();
                isam2_->update();
                isam2_->update();
                isam2_->update();
                loop_found_ = false;
            }
            gtsam_graph_.resize(0);
            initialized_estimate_.clear();

            optimized_estimate_ = isam2_->calculateBestEstimate();
            gtsam::Pose3 latest_estimate = optimized_estimate_.at<gtsam::Pose3>(lastest_index_);
            temp_poses_[lastest_index_].global_rot = latest_estimate.rotation().matrix().cast<double>();
            temp_poses_[lastest_index_].global_pos = latest_estimate.translation().matrix().cast<double>();
            Eigen::Matrix3d offset_rot;
            Eigen::Vector3d offset_pos;
            temp_poses_[lastest_index_].getOffset(offset_rot, offset_pos);

            shared_data_->mutex.lock();
            int current_size = shared_data_->key_poses.size();
            shared_data_->offset_rot = offset_rot;
            shared_data_->offset_pos = offset_pos;
            shared_data_->mutex.unlock();

            for (int i = 0; i < lastest_index_; i++)
            {
                gtsam::Pose3 temp_pose = optimized_estimate_.at<gtsam::Pose3>(i);
                shared_data_->key_poses[i].global_rot = temp_pose.rotation().matrix().cast<double>();
                shared_data_->key_poses[i].global_pos = temp_pose.translation().matrix().cast<double>();
            }

            for (int i = lastest_index_; i < current_size; i++)
            {
                shared_data_->key_poses[i].addOffset(offset_rot, offset_pos);
            }
        }
    };

    class LocalizerThread
    {
    public:
        LocalizerThread() {}

        void setSharedDate(std::shared_ptr<SharedData> shared_data)
        {
            shared_data_ = shared_data;
        }

        void setRate(double rate)
        {
            rate_ = std::make_shared<rclcpp::Rate>(rate);
        }
        void setRate(std::shared_ptr<rclcpp::Rate> rate)
        {
            rate_ = rate;
        }
        void setLocalizer(std::shared_ptr<IG_LIO::IcpLocalizer> localizer)
        {
            icp_localizer_ = localizer;
        }

        void operator()()
        {
            current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);

            while (is_alive)
            {
                rate_->sleep();
                if (terminate_flag)
                {
                    is_alive = !terminate_flag;
                    break;
                }
                if (shared_data_->halt_flag)
                    continue;
                if (!shared_data_->localizer_activate)
                    continue;
                if (!shared_data_->pose_updated)
                    continue;
                gloabl_pose_.setIdentity();
                bool rectify = false;
                Eigen::Matrix4d init_guess;
                {
                    std::lock_guard<std::mutex> lock(shared_data_->mutex);
                    shared_data_->pose_updated = false;
                    init_guess.setIdentity();
                    local_rot_ = shared_data_->local_rot;
                    local_pos_ = shared_data_->local_pos;
                    init_guess.block<3, 3>(0, 0) = shared_data_->offset_rot * local_rot_;
                    init_guess.block<3, 1>(0, 3) = shared_data_->offset_rot * local_pos_ + shared_data_->offset_pos;
                    pcl::copyPointCloud(*shared_data_->cloud, *current_cloud_);
                }

                if (shared_data_->localizer_service_called)
                {
                    std::lock_guard<std::mutex> lock(shared_data_->service_mutex);
                    shared_data_->localizer_service_called = false;
                    icp_localizer_->init(shared_data_->map_path, false);
                    gloabl_pose_ = icp_localizer_->multi_align_sync(current_cloud_, shared_data_->initial_guess);
                    if (icp_localizer_->isSuccess())
                    {
                        rectify = true;
                        shared_data_->localizer_activate = true;
                        shared_data_->localizer_service_success = true;
                    }

                    else
                    {
                        rectify = false;
                        shared_data_->localizer_activate = false;
                        shared_data_->localizer_service_success = false;
                    }
                }
                else
                {
                    gloabl_pose_ = icp_localizer_->align(current_cloud_, init_guess);
                    if (icp_localizer_->isSuccess())
                        rectify = true;
                    else
                        rectify = false;
                }

                if (rectify)
                {
                    std::lock_guard<std::mutex> lock(shared_data_->mutex);
                    shared_data_->offset_rot = gloabl_pose_.block<3, 3>(0, 0) * local_rot_.transpose();
                    shared_data_->offset_pos = -gloabl_pose_.block<3, 3>(0, 0) * local_rot_.transpose() * local_pos_ + gloabl_pose_.block<3, 1>(0, 3);
                }
            }
        }

        void stop()
        {
            terminate_flag = true;
        }

    private:
        bool is_alive = true;
        bool terminate_flag = false;
        std::shared_ptr<SharedData> shared_data_;
        std::shared_ptr<IG_LIO::IcpLocalizer> icp_localizer_;
        std::shared_ptr<rclcpp::Rate> rate_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_;
        Eigen::Matrix4d gloabl_pose_;
        Eigen::Matrix3d local_rot_;
        Eigen::Vector3d local_pos_;
    };

    class MapBuilderNode : public rclcpp::Node
    {
    public:
        MapBuilderNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~MapBuilderNode();

        void run();
        void stop();

        std::function<void(void)> f;

    private:
        void param_respond();
        void initSubscribers();
        void initPublishers();
        void initSerivces();
        void init();
        void systemReset();
        void addKeyPose();
        void publishBodyCloud(const sensor_msgs::msg::PointCloud2 &cloud_to_pub);
        void publishMapCloud(const sensor_msgs::msg::PointCloud2 &cloud_to_pub);
        void publishLocalCloudAndGridMap(const sensor_msgs::msg::PointCloud2 &cloud_to_pub);
        void publishOdom(const nav_msgs::msg::Odometry &odom_to_pub);
        void publishBaseLink();
        void publishLocalPath();
        void publishGlobalPath();
        void publishLoopMark();
        void saveMapCallBack(const ig_lio_c_msgs::srv::SaveMap::Request::SharedPtr request,
                             const ig_lio_c_msgs::srv::SaveMap::Response::SharedPtr response);
        void relocCallback(const ig_lio_c_msgs::srv::ReLoc::Request::SharedPtr request,
                           const ig_lio_c_msgs::srv::ReLoc::Response::SharedPtr response);

    private:
        std::string global_frame_;
        std::string local_frame_;
        std::string body_frame_;
        double current_time_;
        bool publish_map_cloud_;
        IG_LIO::State current_state_;
        ImuData imu_data_;
        LivoxData livox_data_;
        MeasureGroup measure_group_;
        IG_LIO::IGLIOParams lio_params_;
        IG_LIO::LocalizerParams localizer_params_;
        std::shared_ptr<IG_LIO::IGLIOBuilder> lio_builder_;
        std::shared_ptr<IG_LIO::IcpLocalizer> icp_localizer_;
        std::shared_ptr<SharedData> shared_data_;
        std::shared_ptr<rclcpp::Rate> local_rate_;
        std::shared_ptr<rclcpp::Rate> loop_rate_lc_;
        std::shared_ptr<rclcpp::Rate> loop_rate_l_;
        LoopClosureThread loop_closure_;
        std::shared_ptr<std::thread> loop_thread_;
        LocalizerThread localizer_loop_;
        std::shared_ptr<std::thread> localizer_thread_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
        pcl::PCDWriter writer_;
        std::shared_ptr<grid_map::GridMapPclLoader> gridMapPclLoader;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_cloud_pub_;
        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr local_grid_map_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_cloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_mark_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
        rclcpp::Service<ig_lio_c_msgs::srv::SaveMap>::SharedPtr Savemap_Server;
        rclcpp::Service<ig_lio_c_msgs::srv::ReLoc>::SharedPtr Reloc_Server;

        Eigen::Matrix3d offset_rot_ = Eigen::Matrix3d::Identity();

        Eigen::Vector3d offset_pos_ = Eigen::Vector3d::Zero();

        IG_LIO::PointCloudXYZI::Ptr current_cloud_body_;
    };
} // namespace IG_LIO

#endif // _MAP_BUILDER_NODE_H