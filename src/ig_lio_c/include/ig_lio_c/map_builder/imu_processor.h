#ifndef _IMU_PROCESSOR_H
#define _IMU_PROCESSOR_H

#include <mutex>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../ieskf/ieskf.h"
#include "../commons.h"

namespace IG_LIO
{
    struct PiontIMU
    {
    private:
        std::mutex mutex_;
        Eigen::Vector3d pos_;     // 位置
        Eigen::Matrix3d rot_;     // 姿态 旋转矩阵表示
        Eigen::Vector3d w_;       // 角速度
        Eigen::Vector3d v_;       // 线速度
        Eigen::Vector3d gravity_; // 重力加速度

        double lastIMUT_ = 0.; // 上一帧的时间

    public:
        void setlastIMUT(double lastIMUT)
        {
            std::lock_guard<std::mutex> lck(mutex_);
            lastIMUT_ = lastIMUT;
        }

        void setPos(Eigen::Vector3d pos)
        {
            std::lock_guard<std::mutex> lck(mutex_);
            pos_ = pos;
        };

        void setRot(Eigen::Matrix3d rot)
        {
            std::lock_guard<std::mutex> lck(mutex_);
            rot_ = rot;
        };

        void setW(Eigen::Vector3d w)
        {
            std::lock_guard<std::mutex> lck(mutex_);
            w_ = w;
        };

        void setV(Eigen::Vector3d v)
        {
            std::lock_guard<std::mutex> lck(mutex_);
            v_ = v;
        };

        void setGravity(Eigen::Vector3d gravity)
        {
            std::lock_guard<std::mutex> lck(mutex_);
            gravity_ = gravity;
        }

        Eigen::Vector3d getPos()
        {
            std::lock_guard<std::mutex> lck(mutex_);
            return pos_;
        }

        Eigen::Matrix3d getRot()
        {
            std::lock_guard<std::mutex> lck(mutex_);
            return rot_;
        }

        Eigen::Vector3d getW()
        {
            std::lock_guard<std::mutex> lck(mutex_);
            return w_;
        };

        Eigen::Vector3d getV(Eigen::Vector3d v_)
        {
            std::lock_guard<std::mutex> lck(mutex_);
            return v_;
        };

        double getLastIMUT()
        {
            std::lock_guard<std::mutex> lck(mutex_);
            return lastIMUT_;
        }

        void update(IG_LIO::IMU &imu);
    };

    struct Pose
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        Eigen::Matrix3d rot;
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Pose();
        Pose(double t, Eigen::Vector3d a, Eigen::Vector3d g, Eigen::Vector3d v, Eigen::Vector3d p, Eigen::Matrix3d r)
            : offset(t), acc(a), gyro(g), vel(v), pos(p), rot(r) {}
        double offset;
    };

    class IMUProcessor
    {
    public:
        IMUProcessor(std::shared_ptr<IG_LIO::IESKF> kf, std::shared_ptr<IG_LIO::PiontIMU> pointIMU);

        void init(const MeasureGroup &meas);

        void undistortPointcloud(const MeasureGroup &meas, PointCloudXYZI::Ptr &out);

        bool operator()(const MeasureGroup &meas, PointCloudXYZI::Ptr &out);

        bool isInitialized() const { return init_flag_; }

        void setMaxInitCount(int max_init_count) { max_init_count_ = max_init_count; }

        void setExtParams(Eigen::Matrix3d &rot_ext, Eigen::Vector3d &pos_ext);

        void setAccCov(Eigen::Vector3d acc_cov) { acc_cov_ = acc_cov; }

        void setGyroCov(Eigen::Vector3d gyro_cov) { gyro_cov_ = gyro_cov; }

        void setAccBiasCov(Eigen::Vector3d acc_bias_cov) { acc_bias_cov_ = acc_bias_cov; }

        void setGyroBiasCov(Eigen::Vector3d gyro_bias_cov) { gyro_bias_cov_ = gyro_bias_cov; }

        void setCov(Eigen::Vector3d gyro_cov, Eigen::Vector3d acc_cov, Eigen::Vector3d gyro_bias_cov, Eigen::Vector3d acc_bias_cov);

        void setCov(double gyro_cov, double acc_cov, double gyro_bias_cov, double acc_bias_cov);

        void setAlignGravity(bool align_gravity) { align_gravity_ = align_gravity; }

        void setSetInitpose(bool set_initpose) { set_initpose_ = set_initpose; }

        void setInitpose(std::vector<double> ext_r);

        void reset();

    private:
        int init_count_ = 0;
        int max_init_count_ = 30;
        Eigen::Matrix3d rot_ext_;
        Eigen::Vector3d pos_ext_;
        std::vector<double> ext_r_;
        std::shared_ptr<IG_LIO::IESKF> kf_;
        std::shared_ptr<IG_LIO::PiontIMU> pointIMU_;

        IG_LIO::IMU last_imu_;
        bool init_flag_ = false;
        bool align_gravity_ = true;
        bool set_initpose_ = true;

        Eigen::Vector3d mean_acc_;
        Eigen::Vector3d mean_gyro_;

        Eigen::Vector3d last_acc_;
        Eigen::Vector3d last_gyro_;

        std::vector<Pose> imu_poses_;

        double last_lidar_time_end_;

        Eigen::Vector3d gyro_cov_;
        Eigen::Vector3d acc_cov_;
        Eigen::Vector3d gyro_bias_cov_;
        Eigen::Vector3d acc_bias_cov_;

        IG_LIO::Matrix12d Q_;
    };
} // namespace IG_LIO

#endif // _IMU_PROCESSOR_H