#ifndef _OCCUPANCY_GRID_CONERTER_H
#define _OCCUPANCY_GRID_CONERTER_H

#include <queue>
#include <string>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_util/occ_grid_values.hpp>
#include <filters/filter_chain.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ig_lio_c_msgs/srv/covert_map.hpp>

#include <nav2_map_server/map_mode.hpp>
#include <Magick++.h>

#include <tbb/parallel_for.h>

namespace IG_LIO
{
    using namespace std::chrono;
    using std::placeholders::_1;
    using std::placeholders::_2;
    namespace gm = ::grid_map::grid_map_pcl;

    struct SaveParameters
    {
        std::string map_file_name{""};
        std::string image_format{""};
        double free_thresh{0.0};
        double occupied_thresh{0.0};
        nav2_map_server::MapMode mode{nav2_map_server::MapMode::Trinary};
    };

    class OccupancyGridConverterNode : public rclcpp::Node
    {
    public:
        OccupancyGridConverterNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~OccupancyGridConverterNode();

    private:
        void param_respond();
        void initSubscribers();
        void initPublishers();
        void initSerivces();
        void init();
        grid_map::GridMap makeGridMapFromDepth(const sensor_msgs::msg::PointCloud2 &cloud_to_make);
        grid_map::GridMap makeGridMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_to_make);
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void publishGridMap(const grid_map::GridMap &gridMap_to_pub);
        std::shared_ptr<nav_msgs::msg::OccupancyGrid> createOccupancyGridMsg(const grid_map::GridMap &gridMap);
        void publishOccupancyGridMapMap(std::shared_ptr<nav_msgs::msg::OccupancyGrid> &occupancyGrid_to_pub);
        void covertMapCallBack(const ig_lio_c_msgs::srv::CovertMap::Request::SharedPtr request,
                               const ig_lio_c_msgs::srv::CovertMap::Response::SharedPtr response);
        void tryWriteMapToFile(
            const nav_msgs::msg::OccupancyGrid &map,
            const SaveParameters &save_parameters);

    private:
        std::string robot_frame = "base_link";
        int grid_map_cloud_size = 10;
        double occupancyGriddataMin = 0.5;
        double occupancyGriddataMax = 10.0;
        double point_min_dist_ = 0.4;
        double point_max_dist_ = 2.5;
        double min_distance = 0.4;
        std::shared_ptr<grid_map::GridMapPclLoader> gridMapPclLoader;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr local_grid_map_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_map_;
        std::deque<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> grid_map_cloud_;
        rclcpp::Service<ig_lio_c_msgs::srv::CovertMap>::SharedPtr CovertMap_Server;
        std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
        std::shared_ptr<tf2_ros::TransformListener> tfListener_;
        filters::FilterChain<grid_map::GridMap> filterChain_local_;
        filters::FilterChain<grid_map::GridMap> filterChain_map_;
        std::string filterChainParametersName_local_;
        std::string filterChainParametersName_map_;
    };
} // namespace IG_LIO

#endif // _OCCUPANCY_GRID_CONERTER_H