#ifndef _IGLIO_BUILDER_H
#define _IGLIO_BUILDER_H

#include <pcl/common/transforms.h>

#include "../commons.h"
#include "./imu_processor.h"
#include "../ieskf/ieskf.h"
#include "../voxel_map/voxel_map.h"

namespace IG_LIO
{
    struct GICPCorrespond
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d meanA;
        Eigen::Vector3d meanB;
        Eigen::Matrix3d covA;
        Eigen::Matrix3d covB;
        GICPCorrespond(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Matrix3d &ca, const Eigen::Matrix3d &cb) : meanA(a), meanB(b), covA(ca), covB(cb) {}
    };
    struct FASTLIOCorrspond
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d point;
        Eigen::Vector4d plane;
    };
    inline void CauchyLossFunction(const double e, const double delta, Eigen::Vector3d &rho);
    struct IGLIOParams
    {
        double point2plane_gain = 1000.0;
        double gicp_constraint_gain = 100.0;
        double scan_resolution = 0.5;
        double map_resolution = 0.5;
        int max_points_per_scan = 10000;
        size_t map_capacity = 5000000;
        size_t grid_capacity = 20;
        VoxelMap::MODE mode = VoxelMap::MODE::NEARBY_7;
        double esikf_min_iteration = 2;
        double esikf_max_iteration = 30;
        double imu_acc_cov = 0.01;
        double imu_gyro_cov = 0.01;
        double imu_acc_bias_cov = 0.0001;
        double imu_gyro_bias_cov = 0.0001;

        std::vector<double> imu_ext_rot = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        std::vector<double> imu_ext_pos = {-0.011, -0.02329, 0.04412};

        std::vector<double> ext_r = {3.14, 0., 0.};
        std::vector<double> ext_t = {-0.0151, 0., 0.};

        bool extrinsic_est_en = false;
        bool align_gravity = true;
        bool set_initpose = true;
    };
    class IGLIOBuilder
    {
    public:
        IGLIOBuilder(IGLIOParams &params);

        Status currentStatus() const { return status; }

        IG_LIO::State currentState() const { return kf_->x(); }

        void mapping(const MeasureGroup &meas);

        void sharedUpdateFunc(IG_LIO::State &, IG_LIO::SharedState &);

        void fastlioConstraint(IG_LIO::State &, IG_LIO::SharedState &);

        void gicpConstraint(IG_LIO::State &, IG_LIO::SharedState &);

        PointCloudXYZI::Ptr transformToWorld(const PointCloudXYZI::Ptr cloud);

        PointCloudXYZI::Ptr cloudUndistortedLidar() { return cloud_lidar_; }

        PointCloudXYZI::Ptr cloudUndistortedBody();

        PointCloudXYZI::Ptr cloudWorld();

        void reset();

    private:
        IGLIOParams params_;
        Status status = Status::INITIALIZE;
        std::shared_ptr<IMUProcessor> imu_processor_;
        std::shared_ptr<IG_LIO::IESKF> kf_;
        std::shared_ptr<VoxelMap> voxel_map_;
        std::shared_ptr<FastVoxelMap> fast_voxel_map_;
        PointCloudXYZI::Ptr cloud_lidar_;
        PointCloudXYZI::Ptr cloud_body_;
        PointCloudXYZI::Ptr cloud_world_;
        Eigen::Matrix3d key_rot_;
        Eigen::Vector3d key_pos_;
        size_t frame_count_ = 0;
        size_t key_frame_count_ = 0;

        std::vector<PointCov> point_array_lidar_;

        std::vector<FASTLIOCorrspond> fastlio_cache_;
        std::vector<GICPCorrespond> gicp_cache_;
        std::vector<bool> cache_flag_;
    };
} // namespace IG_LIO

#endif // _IGLIO_BUILDER_H