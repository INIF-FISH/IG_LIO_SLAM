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
        declareParameter("global_frame", rclcpp::ParameterValue("odom"));
        declareParameter("map_time_decay", rclcpp::ParameterValue(0.2));
        grid_map_ = std::make_shared<grid_map::GridMap>();
        node->get_parameter(name_ + "." + "map_topic", map_topic_);
        node->get_parameter(name_ + "." + "global_frame", global_frame_);
        node->get_parameter(name_ + "." + "map_time_decay", time_decay_);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        subscriber_ = node->create_subscription<grid_map_msgs::msg::GridMap>(map_topic_,
                                                                             rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
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

        map_lock_.lock();
        grid_map_ = map_ptr;
        map_lock_.unlock();
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

        if (abs(robot_x) < 1e3 && abs(robot_y) < 1e3)
        {
            *min_x = min_x_tmp;
            *min_y = min_y_tmp;
            *max_x = max_x_tmp;
            *max_y = max_y_tmp;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"), "Illegal robot pos detected! X:%.f ,Y:%.f", robot_x, robot_y);
        }
    }

    void MsgLayer::onFootprintChanged()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "MsgLayer::onFootprintChanged(): num footprint points: %lu",
                     layered_costmap_->getFootprint().size());
    }

    void MsgLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        unsigned char *master_array = master_grid.getCharMap();
        unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);
        map_lock_.lock();
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
                    double origin_cost = master_array[index];
                    double slope = grid_map_->atPosition("slope", vec2d_map);
                    double elevation = grid_map_->atPosition("elevation", vec2d_map) + 0.6;
                    if (!std::isnan(elevation))
                    {
                        double cost;
                        if (!std::isnan(slope))
                            cost = slope * 254;
                        else
                            cost = (elevation + 0.2) * 254;

                        if (origin_cost != 0)
                            cost = (0.8 * origin_cost + 0.6 * cost);

                        if (cost > 200)
                            cost = 254;
                        else
                            // cost = 0;
                            continue;

                        master_array[index] = cost;
                    }
                }
            }
        }
        map_lock_.unlock();
    }

} // namespace nav2_msg_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_msg_costmap_plugin::MsgLayer, nav2_costmap_2d::Layer)
