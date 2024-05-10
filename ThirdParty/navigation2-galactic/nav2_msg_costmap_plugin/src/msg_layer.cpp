#include "nav2_msg_costmap_plugin/msg_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_msg_costmap_plugin
{

    MsgLayer::MsgLayer()
        : last_min_x_(-std::numeric_limits<float>::max()),
          last_min_y_(-std::numeric_limits<float>::max()),
          last_max_x_(std::numeric_limits<float>::max()),
          last_max_y_(std::numeric_limits<float>::max())
    {
    }

    void MsgLayer::onInitialize()
    {
        auto node = node_.lock();
        declareParameter("map_topic", rclcpp::ParameterValue("grid_map"));
        declareParameter("global_frame", rclcpp::ParameterValue("map"));
        declareParameter("map_time_decay", rclcpp::ParameterValue(0.2));
        declareParameter("max_slope", rclcpp::ParameterValue(0.5));
        declareParameter("robot_base_frame", rclcpp::ParameterValue("base_link"));
        declareParameter("use_height_offset", rclcpp::ParameterValue(false));
        declareParameter("robot_height_offset", rclcpp::ParameterValue(0.5));
        grid_map_ = std::make_shared<grid_map::GridMap>();
        node->get_parameter(name_ + "." + "map_topic", map_topic_);
        node->get_parameter(name_ + "." + "global_frame", global_frame_);
        node->get_parameter(name_ + "." + "map_time_decay", time_decay_);
        node->get_parameter(name_ + "." + "max_slope", max_slope);
        node->get_parameter(name_ + "." + "robot_base_frame", robot_base_frame_);
        node->get_parameter(name_ + "." + "use_robot_z", use_robot_z_);
        node->get_parameter(name_ + "." + "robot_height_offset", robot_height_offset_);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        subscriber_ = node->create_subscription<grid_map_msgs::msg::GridMap>(map_topic_,
                                                                             rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local(),
                                                                             std::bind(&MsgLayer::callback,
                                                                                       this, std::placeholders::_1));
        need_recalculation_ = false;
        current_ = true;
    }

    void MsgLayer::callback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        auto frame_id = msg->header.frame_id;
        tf2::Transform transform;
        geometry_msgs::msg::TransformStamped tf_msg;
        std::shared_ptr<grid_map::GridMap> map_ptr;

        if (global_frame_ != frame_id)
        {
            try
            {
                tf_msg = tf_buffer_->lookupTransform(global_frame_, frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.2));
                tf2::convert(tf_msg.transform, transform);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_ERROR(rclcpp_node_->get_logger(), "%s", ex.what());
                return;
            }
            double roll, pitch, yaw;
            tf2::Vector3 origin;
            transform.getBasis().getRPY(roll, pitch, yaw);
            origin = transform.getOrigin();
            tf2::Quaternion q_projected;
            q_projected.setRPY(0, 0, yaw);
            origin.setZ(0);
            transform_projected_.setRotation(q_projected);
            transform_projected_.setOrigin(origin);
            geometry_msgs::msg::TransformStamped tf_projected_stamped;
            tf_projected_stamped.header.stamp = msg->header.stamp;
            tf_projected_stamped.header.frame_id = global_frame_;
            tf_projected_stamped.child_frame_id = global_frame_ + "_projected";
            tf_projected_stamped.transform.translation.x = transform_projected_.getOrigin().getX();
            tf_projected_stamped.transform.translation.y = transform_projected_.getOrigin().getY();
            tf_projected_stamped.transform.translation.z = transform_projected_.getOrigin().getZ();
            tf_projected_stamped.transform.rotation.x = transform_projected_.getRotation().x();
            tf_projected_stamped.transform.rotation.y = transform_projected_.getRotation().y();
            tf_projected_stamped.transform.rotation.z = transform_projected_.getRotation().z();
            tf_projected_stamped.transform.rotation.w = transform_projected_.getRotation().w();

            tf_broadcaster_->sendTransform(tf_projected_stamped);
            grid_map::GridMap grid_map;
            grid_map::GridMapRosConverter::fromMessage(*msg, grid_map);
            map_ptr = std::make_shared<grid_map::GridMap>(grid_map.getTransformedMap(tf2::transformToEigen(tf_projected_stamped.transform),
                                                                                     "elevation", global_frame_));
            map_ptr->add("slope", grid_map.getTransformedMap(tf2::transformToEigen(tf_projected_stamped.transform), "slope", global_frame_)["slope"]);
        }
        else
        {
            grid_map::GridMap grid_map;
            grid_map::GridMapRosConverter::fromMessage(*msg, grid_map);
            map_ptr = std::make_shared<grid_map::GridMap>(grid_map);
        }
        {
            std::lock_guard<std::mutex> lck(map_lock_);
            grid_map_ = map_ptr;
            last_timestamp = msg->header.stamp;
        }
    }

    void MsgLayer::updateBounds(
        double robot_x, double robot_y, double robot_yaw, double *min_x,
        double *min_y, double *max_x, double *max_y)
    {
        double min_x_tmp,
            max_x_tmp,
            min_y_tmp,
            max_y_tmp;
        min_x_tmp = robot_x - 3;
        max_x_tmp = robot_x + 3;
        min_y_tmp = robot_y - 3;
        max_y_tmp = robot_y + 3;
        *min_x = min_x_tmp;
        *min_y = min_y_tmp;
        *max_x = max_x_tmp;
        *max_y = max_y_tmp;
        if (!(abs(robot_x) < 1e3 && abs(robot_y) < 1e3))
        {
            RCLCPP_WARN(rclcpp::get_logger("nav2_costmap_2d"), "Might illegal robot pos detected! X:%.f ,Y:%.f", robot_x, robot_y);
        }
    }

    void MsgLayer::onFootprintChanged()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "MsgLayer::onFootprintChanged(): num footprint points: %lu",
                     layered_costmap_->getFootprint().size());
    }

    void MsgLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        double robot_z = 0.0;
        if (use_robot_z_)
        {
            geometry_msgs::msg::TransformStamped tf_global_to_base;
            try
            {
                tf_global_to_base = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, last_timestamp, rclcpp::Duration::from_seconds(0.2));
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"), "%s", ex.what());
                return;
            }
            robot_z = tf_global_to_base.transform.translation.z;
        }
        unsigned char *master_array = master_grid.getCharMap();
        unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);
        {
            std::lock_guard<std::mutex> lck(map_lock_);
            for (int j = min_j; j < max_j; j++)
            {
                for (int i = min_i; i < max_i; i++)
                {
                    double worldx, worldy;
                    master_grid.mapToWorld(i, j, worldx, worldy);
                    Eigen::Vector2d vec2d_map(worldx, worldy);
                    Eigen::Array2i idx_map;
                    int index = master_grid.getIndex(i, j);
                    if (grid_map_->isInside(vec2d_map))
                    {
                        double elevation = grid_map_->atPosition("elevation", vec2d_map);
                        double slope = grid_map_->atPosition("slope", vec2d_map);
                        if (!std::isnan(slope))
                        {
                            double origin_cost = master_array[index];
                            double cost;
                            if (slope > max_slope)
                            {
                                if (use_robot_z_ && abs(elevation + robot_height_offset_ - robot_z) > 0.3)
                                {
                                    cost = 254;
                                }
                                else if (!use_robot_z_)
                                    cost = 254;
                                else
                                    cost = origin_cost;
                            }
                            else
                                cost = origin_cost;

                            master_array[index] = cost;
                        }
                    }
                }
            }
        }
    }

} // namespace nav2_msg_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_msg_costmap_plugin::MsgLayer, nav2_costmap_2d::Layer)
