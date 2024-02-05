#include "../../include/ig_lio_c/commons.h"

namespace IG_LIO
{
    Eigen::Vector3d rotate2rpy(Eigen::Matrix3d &rot)
    {
        double roll = std::atan2(rot(2, 1), rot(2, 2));
        double pitch = asin(-rot(2, 0));
        double yaw = std::atan2(rot(1, 0), rot(0, 0));
        return Eigen::Vector3d(roll, pitch, yaw);
    }

    bool MeasureGroup::syncPackage(ImuData &imu_data, LivoxData &livox_data)
    {
        if (imu_data.buffer.empty() || livox_data.buffer.empty())
            return false;

        if (!lidar_pushed)
        {
            lidar = livox_data.buffer.front();
            lidar_time_begin = livox_data.time_buffer.front();
            lidar_time_end = lidar_time_begin + lidar->points.back().curvature / double(1000);
            lidar_pushed = true;
        }

        if (imu_data.last_timestamp < lidar_time_end)
            return false;
        double imu_time = imu_data.buffer.front().timestamp;
        imus.clear();
        while (!imu_data.buffer.empty() && (imu_time < lidar_time_end))
        {
            imu_time = imu_data.buffer.front().timestamp;
            if (imu_time > lidar_time_end)
                break;
            imus.push_back(imu_data.buffer.front());
            imu_data.buffer.pop_front();
        }
        livox_data.buffer.pop_front();
        livox_data.time_buffer.pop_front();
        lidar_pushed = false;
        return true;
    }

    bool esti_plane(Eigen::Vector4d &out, const PointVector &points, const double &thresh)
    {
        Eigen::Matrix<double, NUM_MATCH_POINTS, 3> A;
        Eigen::Matrix<double, NUM_MATCH_POINTS, 1> b;
        A.setZero();
        b.setOnes();
        b *= -1.0;
        for (int i = 0; i < NUM_MATCH_POINTS; i++)
        {
            A(i, 0) = points[i].x;
            A(i, 1) = points[i].y;
            A(i, 2) = points[i].z;
        }

        Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);

        double norm = normvec.norm();
        out[0] = normvec(0) / norm;
        out[1] = normvec(1) / norm;
        out[2] = normvec(2) / norm;
        out[3] = 1.0 / norm;

        for (int j = 0; j < NUM_MATCH_POINTS; j++)
        {
            if (std::fabs(out(0) * points[j].x + out(1) * points[j].y + out(2) * points[j].z + out(3)) > thresh)
            {
                return false;
            }
        }
        return true;
    }

    bool esti_plane(Eigen::Vector4d &out, const std::vector<Eigen::Vector3d> &points, const double &thresh, bool none)
    {
        if (points.size() < 3)
            return false;

        Eigen::MatrixXd A(points.size(), 3);
        Eigen::VectorXd b(points.size());
        A.setZero();
        b.setOnes();
        b *= -1.0;

        for (int i = 0; i < points.size(); i++)
        {
            A(i, 0) = points[i](0);
            A(i, 1) = points[i](1);
            A(i, 2) = points[i](2);
        }

        Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);

        double norm = normvec.norm();
        out[0] = normvec(0) / norm;
        out[1] = normvec(1) / norm;
        out[2] = normvec(2) / norm;
        out[3] = 1.0 / norm;
        for (int j = 0; j < points.size(); j++)
        {
            if (std::fabs(out(0) * points[j](0) + out(1) * points[j](1) + out(2) * points[j](2) + out(3)) > thresh)
            {
                return false;
            }
        }
        return true;
    }

    float sq_dist(const PointType &p1, const PointType &p2)
    {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    }
} // namespace IG_LIO
