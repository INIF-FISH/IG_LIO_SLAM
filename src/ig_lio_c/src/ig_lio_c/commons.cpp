#include "../../include/ig_lio_c/commons.h"

namespace IG_LIO
{
    void ImuData::callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex);
        double timestamp = double(msg->header.stamp.sec) + double(msg->header.stamp.nanosec) / 1e9;
        if (timestamp < last_timestamp)
        {
            std::cout << "imu loop back, clear buffer, last_timestamp: " << last_timestamp << "  current_timestamp: " << timestamp << std::endl;
            buffer.clear();
        }
        last_timestamp = timestamp;
        buffer.emplace_back(timestamp,
                            msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z,
                            msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z);
    }

    void LivoxData::callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex);
        double timestamp = double(msg->header.stamp.sec) + double(msg->header.stamp.nanosec) / 1e9;
        if (timestamp < last_timestamp)
        {
            std::cout << "livox loop back, clear buffer, last_timestamp: " << last_timestamp << "  current_timestamp: " << timestamp << std::endl;
            buffer.clear();
            time_buffer.clear();
        }
        last_timestamp = timestamp;
        IG_LIO::PointCloudXYZI::Ptr ptr(new IG_LIO::PointCloudXYZI());
        livox2pcl(msg, ptr);
        buffer.push_back(ptr);
        time_buffer.push_back(last_timestamp);
    }

    void LivoxData::livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, IG_LIO::PointCloudXYZI::Ptr out)
    {
        int point_num = msg->point_num;
        out->clear();
        out->reserve(point_num / filter_num + 1);
        uint valid_num = 0;
        for (uint i = 0; i < point_num; i++)
        {
            if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {
                valid_num++;
                if (valid_num % filter_num != 0)
                    continue;
                IG_LIO::PointType p;
                p.x = msg->points[i].x;
                p.y = msg->points[i].y;
                p.z = msg->points[i].z;
                p.intensity = float(msg->points[i].reflectivity);
                p.curvature = float(msg->points[i].offset_time) / float(1000000);
                if ((p.x * p.x + p.y * p.y + p.z * p.z) > (blind * blind))
                {
                    out->push_back(p);
                }
            }
        }
    }

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
            lidar_time_end =  double(lidar_time_begin) + double(lidar->points.back().curvature) / double(1000);
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

    sensor_msgs::msg::PointCloud2 pcl2msg(IG_LIO::PointCloudXYZI::Ptr inp, std::string &frame_id, const double &timestamp)
    {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*inp, msg);
        if (timestamp < 0)
            msg.header.stamp = rclcpp::Clock().now();
        else
            msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(timestamp * 1e9));
        msg.header.frame_id = frame_id;
        return msg;
    }

    nav_msgs::msg::Odometry eigen2Odometry(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos, const std::string &frame_id, const std::string &child_frame_id, const double &timestamp)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = rclcpp::Time(static_cast<uint64_t>(timestamp * 1e9));
        odom.child_frame_id = child_frame_id;
        Eigen::Quaterniond q = Eigen::Quaterniond(rot);
        odom.pose.pose.position.x = pos(0);
        odom.pose.pose.position.y = pos(1);
        odom.pose.pose.position.z = pos(2);

        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        return odom;
    }

    geometry_msgs::msg::TransformStamped eigen2Transform(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos, const std::string &frame_id, const std::string &child_frame_id, const double &timestamp)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = frame_id;
        transform.header.stamp = rclcpp::Time(static_cast<uint64_t>(timestamp * 1e9));
        transform.child_frame_id = child_frame_id;
        transform.transform.translation.x = pos(0);
        transform.transform.translation.y = pos(1);
        transform.transform.translation.z = pos(2);
        Eigen::Quaterniond q = Eigen::Quaterniond(rot);
        transform.transform.rotation.w = q.w();
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        return transform;
    }
} // namespace IG_LIO
