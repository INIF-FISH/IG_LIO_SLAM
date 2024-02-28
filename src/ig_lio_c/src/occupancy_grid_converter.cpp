#include "../include/occupancy_grid_converter.h"

namespace IG_LIO
{
    OccupancyGridConverterNode::OccupancyGridConverterNode(const rclcpp::NodeOptions &options)
        : Node("map_builder", options), filterChain_("grid_map::GridMap")
    {
        param_respond();
        initSubscribers();
        initPublishers();
        initSerivces();
        init();
    }

    OccupancyGridConverterNode::~OccupancyGridConverterNode()
    {
    }

    void OccupancyGridConverterNode::param_respond()
    {
        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<int>("grid_map_cloud_size", 10);
        this->declare_parameter<double>("occupancyGriddataMin", -0.1);
        this->declare_parameter<double>("occupancyGriddataMax", 10.0);
        this->declare_parameter<double>("min_distance", 0.4);
        this->declare_parameter("filter_chain_parameter_name", std::string("filters"));
        this->get_parameter("robot_frame", robot_frame);
        this->get_parameter("grid_map_cloud_size", grid_map_cloud_size);
        this->get_parameter("occupancyGriddataMin", occupancyGriddataMin);
        this->get_parameter("occupancyGriddataMax", occupancyGriddataMax);
        this->get_parameter("min_distance", min_distance);
        this->get_parameter("filter_chain_parameter_name", filterChainParametersName_);
    }

    void OccupancyGridConverterNode::initSubscribers()
    {
        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "local_cloud", rclcpp::QoS(100).transient_local(), std::bind(&OccupancyGridConverterNode::pointCloudCallback, this, std::placeholders::_1));
    }

    void OccupancyGridConverterNode::initPublishers()
    {
        local_grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
            "/local_costmap/grid_map", rclcpp::QoS(100).transient_local());
        occupancy_grid_pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(100).transient_local());
    }

    void OccupancyGridConverterNode::initSerivces()
    {
        CovertMap_Server = this->create_service<ig_lio_c_msgs::srv::CovertMap>("CovertMap",
                                                                               std::bind(&OccupancyGridConverterNode::covertMapCallBack, this, _1, _2),
                                                                               rmw_qos_profile_services_default);
    }

    void OccupancyGridConverterNode::init()
    {
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
        gridMapPclLoader = std::make_shared<grid_map::GridMapPclLoader>(this->get_logger());
        gridMapPclLoader->loadParameters(gm::getParameterPath());
        if (filterChain_.configure(
                filterChainParametersName_, this->get_node_logging_interface(),
                this->get_node_parameters_interface()))
        {
            RCLCPP_INFO(this->get_logger(), "Filter chain configured.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Could not configure the filter chain!");
            rclcpp::shutdown();
            return;
        }
    }

    grid_map::GridMap OccupancyGridConverterNode::makeGridMapFromDepth(const sensor_msgs::msg::PointCloud2 &cloud_to_make)
    {
        pcl::PointCloud<pcl::PointXYZ> in_cloud;
        pcl::fromROSMsg(cloud_to_make, in_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr = in_cloud.makeShared();
        geometry_msgs::msg::TransformStamped tf_map_to_base;
        try
        {
            tf_map_to_base = tfBuffer_->lookupTransform("map", "base_link", cloud_to_make.header.stamp, rclcpp::Duration::from_seconds(0.1));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            grid_map::GridMap result;
            return result;
        }
        Eigen::Vector3d xyz_base = {tf_map_to_base.transform.translation.x,
                                    tf_map_to_base.transform.translation.y,
                                    tf_map_to_base.transform.translation.z};
        RCLCPP_INFO_STREAM(this->get_logger(), "X: " << tf_map_to_base.transform.translation.x);
        RCLCPP_INFO_STREAM(this->get_logger(), "Y: " << tf_map_to_base.transform.translation.y);
        Eigen::Vector2d xy_base = {tf_map_to_base.transform.translation.x,
                                   tf_map_to_base.transform.translation.y};
        for (size_t i = 0; i < cloud_ptr->points.size(); i++)
        {
            Eigen::Vector3d xyz_pt = {cloud_ptr->points[i].x,
                                      cloud_ptr->points[i].y,
                                      cloud_ptr->points[i].z};
            Eigen::Vector3d dxyz = xyz_pt - xyz_base;
            if (dxyz.norm() > point_min_dist_ && dxyz.norm() < point_max_dist_)
                (*cloud_filtered).points.push_back(cloud_ptr->points[i]);
        }
        if (grid_map_cloud_.size() == grid_map_cloud_size)
            grid_map_cloud_.pop_front();
        grid_map_cloud_.push_back(cloud_filtered);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto ptr : grid_map_cloud_)
        {
            *temp_cloud += *ptr;
        }
        gridMapPclLoader->setInputCloud(temp_cloud);
        gridMapPclLoader->initializeGridMapGeometryFromInputCloud();
        gridMapPclLoader->addLayerFromInputCloud("elevation");
        auto gridMap = gridMapPclLoader->getGridMap();

        gridMap.setFrameId("map");
        gridMap.setTimestamp(this->get_clock()->now().nanoseconds());
        grid_map::GridMap outputMap;
        if (!filterChain_.update(gridMap, outputMap))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain!");
            return gridMap;
        }
        grid_map::Size size = outputMap.getSize();
        for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator)
        {
            grid_map::Index index = *iterator;
            Eigen::Vector2d position;
            outputMap.getPosition(index, position);
            if ((position - xy_base).norm() < min_distance)
            {
                outputMap.at("slope", index) = 0.;
            }
        }
        return outputMap;
    }

    grid_map::GridMap OccupancyGridConverterNode::makeGridMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_to_make)
    {
        gridMapPclLoader->setInputCloud(cloud_to_make);
        gridMapPclLoader->initializeGridMapGeometryFromInputCloud();
        gridMapPclLoader->addLayerFromInputCloud("elevation");
        auto gridMap = gridMapPclLoader->getGridMap();
        gridMap.setFrameId("map");
        gridMap.setTimestamp(this->get_clock()->now().nanoseconds());
        return gridMap;
    }

    void OccupancyGridConverterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message. Height: %d, Width: %d", msg->height, msg->width);
        auto gridMap = makeGridMapFromDepth(*msg);
        publishGridMap(gridMap);
    }

    void OccupancyGridConverterNode::publishGridMap(const grid_map::GridMap &gridMap_to_pub)
    {
        if (local_grid_map_pub_->get_subscription_count() != 0)
        {
            auto msg = grid_map::GridMapRosConverter::toMessage(gridMap_to_pub);
            local_grid_map_pub_->publish(std::move(msg));
        }
    }

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> OccupancyGridConverterNode::createOccupancyGridMsg(const grid_map::GridMap &gridMap)
    {
        auto time = this->get_clock()->now();
        nav_msgs::msg::OccupancyGrid occupancy_grid;
        occupancy_grid.header.frame_id = gridMap.getFrameId();
        occupancy_grid.header.stamp = rclcpp::Time(gridMap.getTimestamp());
        occupancy_grid.info.map_load_time = occupancy_grid.header.stamp;
        occupancy_grid.info.resolution = gridMap.getResolution();
        occupancy_grid.info.width = gridMap.getSize()(0);
        occupancy_grid.info.height = gridMap.getSize()(1);
        grid_map::Position position = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
        occupancy_grid.info.origin.position.x = position.x();
        occupancy_grid.info.origin.position.y = position.y();
        occupancy_grid.info.origin.position.z = 0.0;
        occupancy_grid.info.origin.orientation.x = 0.0;
        occupancy_grid.info.origin.orientation.y = 0.0;
        occupancy_grid.info.origin.orientation.z = 0.0;
        occupancy_grid.info.origin.orientation.w = 1.0;
        size_t nCells = gridMap.getSize().prod();
        occupancy_grid.data.resize(nCells);
        for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator)
        {
            float value = gridMap.at("elevation", *iterator);
            if (std::isnan(value))
            {
                value = nav2_util::OCC_GRID_UNKNOWN;
            }
            else
            {
                if (value > occupancyGriddataMin && value < occupancyGriddataMax)
                    value = nav2_util::OCC_GRID_OCCUPIED;
                else if (value > occupancyGriddataMax)
                    value = nav2_util::OCC_GRID_UNKNOWN;
                else
                    value = nav2_util::OCC_GRID_FREE;
            }
            size_t index = grid_map::getLinearIndexFromIndex(iterator.getUnwrappedIndex(), gridMap.getSize(), false);
            occupancy_grid.data[nCells - index - 1] = value;
        }
        return std::make_shared<nav_msgs::msg::OccupancyGrid>(occupancy_grid);
    }

    void OccupancyGridConverterNode::publishOccupancyGridMapMap(std::shared_ptr<nav_msgs::msg::OccupancyGrid> &occupancyGrid_to_pub)
    {
        if (this->occupancy_grid_pub_map_->get_subscription_count() != 0)
        {
            this->occupancy_grid_pub_map_->publish(*occupancyGrid_to_pub);
        }
    }

    void OccupancyGridConverterNode::covertMapCallBack(const ig_lio_c_msgs::srv::CovertMap::Request::SharedPtr request,
                                                       const ig_lio_c_msgs::srv::CovertMap::Response::SharedPtr response)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(request->pcd_path, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read PCD file\n");
            response->status = 0;
            response->message = "Failed to read PCD file";
            return;
        }
        auto gridMap = makeGridMap(cloud);
        auto occ_grid = createOccupancyGridMsg(gridMap);
        occ_grid->header.frame_id = "map";
        publishOccupancyGridMapMap(occ_grid);
        response->status = 1;
        response->message = "Success to corvert";
    }
} // namespace IG_LIO

bool terminate_flag = false;

void signalHandler(int signum)
{
    std::cout << "SHUTTING DOWN CONVERTER NODE!" << std::endl;
    terminate_flag = true;
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IG_LIO::OccupancyGridConverterNode>();
    node->get_logger().set_level(rclcpp::Logger::Level::Error);
    while (rclcpp::ok() && !terminate_flag)
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(IG_LIO::OccupancyGridConverterNode)
