#include "../include/map_builder_node.h"

namespace IG_LIO
{
    MapBuilderNode::MapBuilderNode(const rclcpp::NodeOptions &options)
        : Node("map_builder", options)
    {
        param_respond();
        initSubscribers();
        initPublishers();
        init();
    }

    MapBuilderNode::~MapBuilderNode()
    {
    }

    void MapBuilderNode::param_respond()
    {
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("local_frame", "local");
        this->declare_parameter<std::string>("body_frame", "body");
        this->declare_parameter<std::string>("imu_topic", "/livox/imu");
        this->declare_parameter<std::string>("livox_topic", "/livox/lidar");
        this->get_parameter("map_frame", global_frame_);
        this->get_parameter("local_frame", local_frame_);
        this->get_parameter("body_frame", body_frame_);
        this->get_parameter("imu_topic", imu_data_.topic);
        this->get_parameter("livox_topic", livox_data_.topic);
        double local_rate, loop_rate;
        this->declare_parameter<double>("local_rate", 20.0);
        this->declare_parameter<double>("loop_rate", 1.0);
        this->get_parameter("local_rate", local_rate);
        this->get_parameter("loop_rate", loop_rate);
        local_rate_ = std::make_shared<rclcpp::Rate>(local_rate);
        loop_rate_ = std::make_shared<rclcpp::Rate>(loop_rate);
        this->declare_parameter<double>("lio_builder/scan_resolution", 0.3);
        this->declare_parameter<double>("lio_builder/map_resolution", 0.3);
        this->declare_parameter<double>("lio_builder/point2plane_gain", 100.0);
        this->declare_parameter<double>("lio_builder/gicp_constraint_gain", 100.0);
        this->get_parameter("lio_builder/scan_resolution", lio_params_.scan_resolution);
        this->get_parameter("lio_builder/map_resolution", lio_params_.map_resolution);
        this->get_parameter("lio_builder/point2plane_gain", lio_params_.point2plane_gain);
        this->get_parameter("lio_builder/gicp_constraint_gain", lio_params_.gicp_constraint_gain);
        int map_capacity, grid_capacity;
        this->declare_parameter<int>("lio_builder/map_capacity", 5000000);
        this->declare_parameter<int>("lio_builder/grid_capacity", 20);
        this->get_parameter("lio_builder/map_capacity", map_capacity);
        this->get_parameter("lio_builder/grid_capacity", grid_capacity);
        lio_params_.map_capacity = static_cast<size_t>(map_capacity);
        lio_params_.grid_capacity = static_cast<size_t>(grid_capacity);
        this->declare_parameter<bool>("lio_builder/align_gravity", true);
        this->declare_parameter<bool>("lio_builder/extrinsic_est_en", false);
        std::vector<double> pre_rot = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        std::vector<double> pre_pos = {-0.011, -0.02329, 0.04412};
        this->declare_parameter<double>("lio_builder/acc_cov", 0.1);
        this->declare_parameter<double>("lio_builder/gyr_cov", 0.1);
        this->declare_parameter<double>("lio_builder/ba_cov", 0.00001);
        this->declare_parameter<double>("lio_builder/bg_cov", 0.00001);
        this->declare_parameter<std::vector<double>>("lio_builder/imu_ext_rot", pre_rot);
        this->declare_parameter<std::vector<double>>("lio_builder/imu_ext_pos", pre_pos);
        this->get_parameter("lio_builder/acc_cov", lio_params_.imu_acc_cov);
        this->get_parameter("lio_builder/gyr_cov", lio_params_.imu_gyro_cov);
        this->get_parameter("lio_builder/ba_cov", lio_params_.imu_acc_bias_cov);
        this->get_parameter("lio_builder/bg_cov", lio_params_.imu_gyro_bias_cov);
        this->get_parameter("lio_builder/align_gravity", lio_params_.align_gravity);
        this->get_parameter("lio_builder/extrinsic_est_en", lio_params_.extrinsic_est_en);
        this->get_parameter("lio_builder/imu_ext_rot", lio_params_.imu_ext_rot);
        this->get_parameter("lio_builder/imu_ext_pos", lio_params_.imu_ext_pos);
        int mode;
        this->declare_parameter<int>("lio_builder/near_mode", 2);
        this->get_parameter("lio_builder/near_mode", mode);
        switch (mode)
        {
        case 1:
            lio_params_.mode = IG_LIO::VoxelMap::MODE::NEARBY_1;
            break;
        case 2:
            lio_params_.mode = IG_LIO::VoxelMap::MODE::NEARBY_7;
            break;
        case 3:
            lio_params_.mode = IG_LIO::VoxelMap::MODE::NEARBY_19;
            break;
        case 4:
            lio_params_.mode = IG_LIO::VoxelMap::MODE::NEARBY_26;
            break;

        default:
            lio_params_.mode = IG_LIO::VoxelMap::MODE::NEARBY_1;
            break;
        }
        std::vector<double> pre_ext_r = {3.14, 0., 0.};
        std::vector<double> pre_ext_t = {-0.0151, 0., 0.};
        this->declare_parameter<std::vector<double>>("lio_slam/ext_r", pre_ext_r);
        this->declare_parameter<std::vector<double>>("lio_slam/ext_t", pre_ext_t);
        this->get_parameter("lio_slam/ext_r", lio_params_.ext_r);
        this->get_parameter("lio_slam/ext_t", lio_params_.ext_t);
        this->declare_parameter<bool>("loop_closure/activate", true);
        this->declare_parameter<double>("loop_closure/rad_thresh", 0.4);
        this->declare_parameter<double>("loop_closure/dist_thresh", 2.5);
        this->declare_parameter<double>("loop_closure/time_thresh", 30.0);
        this->declare_parameter<double>("loop_closure/loop_pose_search_radius", 10.0);
        this->declare_parameter<int>("loop_closure/loop_pose_index_thresh", 5);
        this->declare_parameter<double>("loop_closure/submap_resolution", 0.2);
        this->declare_parameter<int>("loop_closure/submap_search_num", 20);
        this->declare_parameter<double>("loop_closure/loop_icp_thresh", 0.3);
        this->declare_parameter<bool>("loop_closure/z_prior", false);
        this->get_parameter("loop_closure/activate", loop_closure_.mutableParams().activate);
        this->get_parameter("loop_closure/rad_thresh", loop_closure_.mutableParams().rad_thresh);
        this->get_parameter("loop_closure/dist_thresh", loop_closure_.mutableParams().dist_thresh);
        this->get_parameter("loop_closure/time_thresh", loop_closure_.mutableParams().time_thresh);
        this->get_parameter("loop_closure/loop_pose_search_radius", loop_closure_.mutableParams().loop_pose_search_radius);
        this->get_parameter("loop_closure/loop_pose_index_thresh", loop_closure_.mutableParams().loop_pose_index_thresh);
        this->get_parameter("loop_closure/submap_resolution", loop_closure_.mutableParams().submap_resolution);
        this->get_parameter("loop_closure/submap_search_num", loop_closure_.mutableParams().submap_search_num);
        this->get_parameter("loop_closure/loop_icp_thresh", loop_closure_.mutableParams().loop_icp_thresh);
        this->get_parameter("loop_closure/z_prior", loop_closure_.mutableParams().z_prior);
    }

    void MapBuilderNode::initSubscribers()
    {
        rclcpp::QoS qos(1);
        qos.reliability();
        qos.keep_last(1);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_data_.topic, qos, std::bind(&ImuData::callback, &imu_data_, _1));
        livox_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(livox_data_.topic, qos, std::bind(&LivoxData::callback, &livox_data_, _1));
    }

    void MapBuilderNode::initPublishers()
    {
        rclcpp::QoS qos(1);
        qos.reliability();
        qos.keep_last(1);

        local_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_cloud", qos);
        body_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", qos);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", qos);
        loop_mark_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("loop_mark", qos);
        local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", qos);
        global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", qos);
    }

    void MapBuilderNode::init()
    {
        shared_data_ = std::make_shared<SharedData>();
        br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        lio_builder_ = std::make_shared<IG_LIO::IGLIOBuilder>(lio_params_);
        loop_closure_.setRate(loop_rate_);
        loop_closure_.setShared(shared_data_);
        loop_closure_.init();
        loop_thread_ = std::make_shared<std::thread>(std::ref(loop_closure_));
        f = std::bind(&MapBuilderNode::run, this);
        publishBaseLink();
    }

    void MapBuilderNode::run()
    {
        local_rate_->sleep();
        if (!measure_group_.syncPackage(imu_data_, livox_data_))
            return;
        lio_builder_->mapping(measure_group_);
        if (lio_builder_->currentStatus() == IG_LIO::Status::INITIALIZE)
            return;
        current_time_ = measure_group_.lidar_time_end;
        current_state_ = lio_builder_->currentState();
        std::cout << "ba: " << current_state_.ba.transpose()
                  << " ba_norm: " << current_state_.ba.norm()
                  << " bg: " << current_state_.bg.transpose() * 180.0 / M_PI
                  << " bg_norm: " << current_state_.bg.norm() * 180.0 / M_PI
                  << std::endl;
        br_->sendTransform(eigen2Transform(shared_data_->offset_rot,
                                           shared_data_->offset_pos,
                                           global_frame_,
                                           local_frame_,
                                           current_time_));
        br_->sendTransform(eigen2Transform(current_state_.rot,
                                           current_state_.pos,
                                           local_frame_,
                                           body_frame_,
                                           current_time_));

        publishOdom(eigen2Odometry(current_state_.rot,
                                   current_state_.pos,
                                   local_frame_,
                                   body_frame_,
                                   current_time_));

        addKeyPose();

        publishBodyCloud(pcl2msg(lio_builder_->cloudUndistortedBody(),
                                 body_frame_,
                                 current_time_));
        publishLocalCloud(pcl2msg(lio_builder_->cloudWorld(),
                                  local_frame_,
                                  current_time_));
        publishLocalPath();
        publishGlobalPath();
        publishLoopMark();
    }

    void MapBuilderNode::stop()
    {
        loop_closure_.stop();
        loop_thread_->join();
    }

    void MapBuilderNode::addKeyPose()
    {
        int idx = shared_data_->key_poses.size();
        if (shared_data_->key_poses.empty())
        {
            std::lock_guard<std::mutex> lock(shared_data_->mutex);
            shared_data_->key_poses.emplace_back(idx, current_time_, current_state_.rot, current_state_.pos);
            shared_data_->key_poses.back().addOffset(shared_data_->offset_rot, shared_data_->offset_pos);
            shared_data_->key_pose_added = true;
            shared_data_->cloud_history.push_back(lio_builder_->cloudUndistortedBody());
            return;
        }
        Pose6D &last_key_pose = shared_data_->key_poses.back();
        Eigen::Matrix3d diff_rot = last_key_pose.local_rot.transpose() * current_state_.rot;
        Eigen::Vector3d diff_pose = last_key_pose.local_rot.transpose() * (current_state_.pos - last_key_pose.local_pos);
        Eigen::Vector3d rpy = rotate2rpy(diff_rot);
        if (diff_pose.norm() > loop_closure_.mutableParams().dist_thresh ||
            std::abs(rpy(0)) > loop_closure_.mutableParams().rad_thresh ||
            std::abs(rpy(1)) > loop_closure_.mutableParams().rad_thresh ||
            std::abs(rpy(2)) > loop_closure_.mutableParams().rad_thresh)
        {
            std::lock_guard<std::mutex> lock(shared_data_->mutex);
            shared_data_->key_poses.emplace_back(idx, current_time_, current_state_.rot, current_state_.pos);
            shared_data_->key_poses.back().addOffset(shared_data_->offset_rot, shared_data_->offset_pos);
            shared_data_->key_pose_added = true;
            shared_data_->cloud_history.push_back(lio_builder_->cloudUndistortedBody());
        }
    }

    void MapBuilderNode::publishBodyCloud(const sensor_msgs::msg::PointCloud2 &cloud_to_pub)
    {
        if (body_cloud_pub_->get_subscription_count() == 0)
            return;
        body_cloud_pub_->publish(cloud_to_pub);
    }

    void MapBuilderNode::publishLocalCloud(const sensor_msgs::msg::PointCloud2 &cloud_to_pub)
    {
        if (local_cloud_pub_->get_subscription_count() == 0)
            return;
        local_cloud_pub_->publish(cloud_to_pub);
    }

    void MapBuilderNode::publishBaseLink()
    {
        Eigen::Vector3d baselink2odom_t;
        Eigen::Matrix3d baselink2odom_r;
        baselink2odom_r = Eigen::AngleAxisd(lio_params_.ext_r[0], Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(lio_params_.ext_r[1], Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(lio_params_.ext_r[2], Eigen::Vector3d::UnitZ());
        baselink2odom_t << lio_params_.ext_t[0], lio_params_.ext_t[1], lio_params_.ext_t[2];
        Eigen::Isometry3d baselink_pose = Eigen::Isometry3d::Identity();
        baselink_pose.translation() = baselink2odom_t;
        baselink_pose.rotate(baselink2odom_r);
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = body_frame_;
        transform.header.stamp = rclcpp::Clock().now();
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = baselink_pose.translation().x();
        transform.transform.translation.y = baselink_pose.translation().y();
        transform.transform.translation.z = baselink_pose.translation().z();
        Eigen::Quaterniond q = Eigen::Quaterniond(baselink_pose.rotation());
        transform.transform.rotation.w = q.w();
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        static_br_->sendTransform(transform);
    }

    void MapBuilderNode::publishOdom(const nav_msgs::msg::Odometry &odom_to_pub)
    {
        if (odom_pub_->get_subscription_count() == 0)
            return;
        odom_pub_->publish(odom_to_pub);
    }

    void MapBuilderNode::publishLocalPath()
    {
        if (local_path_pub_->get_subscription_count() == 0)
            return;

        if (shared_data_->key_poses.empty())
            return;

        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = rclcpp::Time(static_cast<uint64_t>(current_time_ * 1e9));
        for (Pose6D &p : shared_data_->key_poses)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = global_frame_;
            pose.header.stamp = rclcpp::Time(static_cast<uint64_t>(current_time_ * 1e9));
            pose.pose.position.x = p.local_pos(0);
            pose.pose.position.y = p.local_pos(1);
            pose.pose.position.z = p.local_pos(2);
            Eigen::Quaterniond q(p.local_rot);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            path.poses.push_back(pose);
        }
        local_path_pub_->publish(path);
    }

    void MapBuilderNode::publishGlobalPath()
    {
        if (global_path_pub_->get_subscription_count() == 0)
            return;

        if (shared_data_->key_poses.empty())
            return;
        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = rclcpp::Time(static_cast<uint64_t>(current_time_ * 1e9));
        for (Pose6D &p : shared_data_->key_poses)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = global_frame_;
            pose.header.stamp = rclcpp::Time(static_cast<uint64_t>(current_time_ * 1e9));
            pose.pose.position.x = p.global_pos(0);
            pose.pose.position.y = p.global_pos(1);
            pose.pose.position.z = p.global_pos(2);
            Eigen::Quaterniond q(p.global_rot);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            path.poses.push_back(pose);
        }
        global_path_pub_->publish(path);
    }

    void MapBuilderNode::publishLoopMark()
    {
        if (loop_mark_pub_->get_subscription_count() == 0)
            return;
        if (shared_data_->loop_history.empty())
            return;
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker nodes_marker;

        nodes_marker.header.frame_id = global_frame_;
        nodes_marker.header.stamp = rclcpp::Time(static_cast<uint64_t>(current_time_ * 1e9));
        nodes_marker.ns = "loop_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.1;
        nodes_marker.scale.y = 0.1;
        nodes_marker.scale.z = 0.1;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        visualization_msgs::msg::Marker edges_marker;
        edges_marker.header.frame_id = global_frame_;
        edges_marker.header.stamp = rclcpp::Time(static_cast<uint64_t>(current_time_ * 1e9));
        edges_marker.ns = "loop_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.05;

        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;
        for (auto &p : shared_data_->loop_history)
        {
            Pose6D &p1 = shared_data_->key_poses[p.first];
            Pose6D &p2 = shared_data_->key_poses[p.second];
            geometry_msgs::msg::Point point1;
            point1.x = p1.global_pos(0);
            point1.y = p1.global_pos(1);
            point1.z = p1.global_pos(2);
            geometry_msgs::msg::Point point2;
            point2.x = p2.global_pos(0);
            point2.y = p2.global_pos(1);
            point2.z = p2.global_pos(2);
            nodes_marker.points.push_back(point1);
            nodes_marker.points.push_back(point2);
            edges_marker.points.push_back(point1);
            edges_marker.points.push_back(point2);
        }
        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        loop_mark_pub_->publish(marker_array);
    }
} // namespace IG_LIO

bool terminate_flag = false;

void signalHandler(int signum)
{
    std::cout << "SHUTTING DOWN MAPPING NODE!" << std::endl;
    terminate_flag = true;
}

int main(int argc, char **argv)
{
    std::cout.fill('0');
    std::cout.width(8);
    signal(SIGINT, signalHandler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IG_LIO::MapBuilderNode>();
    while (rclcpp::ok() && !terminate_flag)
    {
        rclcpp::spin_some(node);
        node->f();
    }
    node->stop();
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(IG_LIO::MapBuilderNode)