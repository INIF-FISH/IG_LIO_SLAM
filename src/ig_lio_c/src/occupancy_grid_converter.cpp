#include "../include/occupancy_grid_converter.h"

namespace IG_LIO
{
    OccupancyGridConverterNode::OccupancyGridConverterNode(const rclcpp::NodeOptions &options)
        : Node("map_builder", options), filterChain_local_("grid_map::GridMap"), filterChain_map_("grid_map::GridMap")
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
        this->declare_parameter("filter_chain_parameter_name_local", std::string("filters_local"));
        this->declare_parameter("filter_chain_parameter_name_map", std::string("filters_map"));
        this->get_parameter("robot_frame", robot_frame);
        this->get_parameter("grid_map_cloud_size", grid_map_cloud_size);
        this->get_parameter("occupancyGriddataMin", occupancyGriddataMin);
        this->get_parameter("occupancyGriddataMax", occupancyGriddataMax);
        this->get_parameter("min_distance", min_distance);
        this->get_parameter("filter_chain_parameter_name_local", filterChainParametersName_local_);
        this->get_parameter("filter_chain_parameter_name_map", filterChainParametersName_map_);
    }

    void OccupancyGridConverterNode::initSubscribers()
    {
        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "local_cloud", rclcpp::QoS(10).transient_local().keep_last(1), std::bind(&OccupancyGridConverterNode::pointCloudCallback, this, std::placeholders::_1));
    }

    void OccupancyGridConverterNode::initPublishers()
    {
        local_grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
            "/local_costmap/grid_map", rclcpp::QoS(10).transient_local().keep_last(1));
        occupancy_grid_pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(10).reliable().transient_local().keep_last(1));
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
        if (filterChain_local_.configure(
                filterChainParametersName_local_, this->get_node_logging_interface(),
                this->get_node_parameters_interface()))
        {
            RCLCPP_INFO(this->get_logger(), "Filter chain local configured.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Could not configure the filter chain local!");
            rclcpp::shutdown();
            return;
        }
        if (filterChain_map_.configure(
                filterChainParametersName_map_, this->get_node_logging_interface(),
                this->get_node_parameters_interface()))
        {
            RCLCPP_INFO(this->get_logger(), "Filter chain map configured.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Could not configure the filter chain map!");
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
        Eigen::Vector2d xy_base = {tf_map_to_base.transform.translation.x,
                                   tf_map_to_base.transform.translation.y};
        std::mutex mtx;
        tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud_ptr->points.size()),
                          [&](const tbb::blocked_range<size_t> &range)
                          {
                              for (size_t i = range.begin(); i != range.end(); ++i)
                              {
                                  Eigen::Vector3d xyz_pt = {cloud_ptr->points[i].x,
                                                            cloud_ptr->points[i].y,
                                                            cloud_ptr->points[i].z};
                                  Eigen::Vector3d dxyz = xyz_pt - xyz_base;
                                  if (dxyz.norm() > point_min_dist_ && dxyz.norm() < point_max_dist_)
                                  {
                                      std::lock_guard<std::mutex> lck(mtx);
                                      (*cloud_filtered).points.push_back(cloud_ptr->points[i]);
                                  }
                              }
                          });
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
        if (!filterChain_local_.update(gridMap, outputMap))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain local!");
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
        grid_map::GridMap outputMap;
        if (!filterChain_map_.update(gridMap, outputMap))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain map!");
            return gridMap;
        }
        return outputMap;
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
            float value = gridMap.at("slope", *iterator);
            if (std::isnan(value))
            {
                value = nav2_util::OCC_GRID_UNKNOWN;
            }
            else
            {
                if (value < occupancyGriddataMin)
                    value = nav2_util::OCC_GRID_FREE;
                else if (value > occupancyGriddataMin && value < occupancyGriddataMax)
                    value = nav2_util::OCC_GRID_OCCUPIED;
                else
                    value = nav2_util::OCC_GRID_UNKNOWN;
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
        Magick::InitializeMagick(nullptr);
        if (request->pcd_path.empty())
        {
            PCL_ERROR("PCD file_path empty !\n");
            response->status = 0;
            response->message = "Failed to read PCD file";
            return;
        }
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
        SaveParameters save_parameters;
        {
            save_parameters.image_format = request->image_format;
        }
        if (request->map_file_name.empty())
        {
            std::cout << "Non map_file_name, using map." << std::endl;
            save_parameters.map_file_name = "map";
        }
        else
        {
            save_parameters.map_file_name = request->map_file_name;
        }
        if (request->occupied_thresh == 0.0)
        {
            std::cout << "Non occupied_thresh, using 0.65." << std::endl;
            save_parameters.occupied_thresh = 0.65;
        }
        else
        {
            save_parameters.occupied_thresh = request->occupied_thresh;
        }
        if (request->free_thresh == 0.0)
        {
            std::cout << "Non free_thresh, using 0.25." << std::endl;
            save_parameters.free_thresh = 0.25;
        }
        else
        {
            save_parameters.occupied_thresh = request->occupied_thresh;
        }
        try
        {
            save_parameters.mode = nav2_map_server::map_mode_from_string(request->map_mode);
        }
        catch (std::invalid_argument &)
        {
            save_parameters.mode = nav2_map_server::MapMode::Trinary;
            std::cout << "Map mode parameter not recognized: " << request->map_mode.c_str() << ", using default value (trinary)" << std::endl;
        }
        if (save_parameters.image_format == "")
        {
            save_parameters.image_format = save_parameters.mode == nav2_map_server::MapMode::Scale ? "png" : "pgm";
            std::cout << "[WARN] [covertMapCallBack]: Image format unspecified. Setting it to: " << save_parameters.image_format << std::endl;
        }

        std::transform(
            save_parameters.image_format.begin(),
            save_parameters.image_format.end(),
            save_parameters.image_format.begin(),
            [](unsigned char c)
            { return std::tolower(c); });

        const std::vector<std::string> BLESSED_FORMATS{"bmp", "pgm", "png"};
        if (
            std::find(BLESSED_FORMATS.begin(), BLESSED_FORMATS.end(), save_parameters.image_format) ==
            BLESSED_FORMATS.end())
        {
            std::stringstream ss;
            bool first = true;
            for (auto &format_name : BLESSED_FORMATS)
            {
                if (!first)
                {
                    ss << ", ";
                }
                ss << "'" << format_name << "'";
                first = false;
            }
            std::cout << "[WARN] [covertMapCallBack]: Requested image format '" << save_parameters.image_format << "' is not one of the recommended formats: " << ss.str() << std::endl;
        }
        const std::string FALLBACK_FORMAT = "png";

        try
        {
            Magick::CoderInfo info(save_parameters.image_format);
            if (!info.isWritable())
            {
                std::cout << "[WARN] [covertMapCallBack]: Format '" << save_parameters.image_format << "' is not writable. Using '" << FALLBACK_FORMAT << "' instead" << std::endl;
                save_parameters.image_format = FALLBACK_FORMAT;
            }
        }
        catch (Magick::ErrorOption &e)
        {
            std::cout << "[WARN] [covertMapCallBack]: Format '" << save_parameters.image_format << "' is not usable. Using '" << FALLBACK_FORMAT << "' instead:" << std::endl
                      << e.what() << std::endl;
            save_parameters.image_format = FALLBACK_FORMAT;
        }

        if (
            save_parameters.mode == nav2_map_server::MapMode::Scale &&
            (save_parameters.image_format == "pgm" ||
             save_parameters.image_format == "jpg" ||
             save_parameters.image_format == "jpeg"))
        {
            std::cout << "[WARN] [covertMapCallBack]: Map mode 'scale' requires transparency, but format '" << save_parameters.image_format << "' does not support it. Consider switching image format to 'png'." << std::endl;
        }
        tryWriteMapToFile(*occ_grid, save_parameters);
        publishOccupancyGridMapMap(occ_grid);
        response->status = 1;
        response->message = "Success to corvert";
    }

    void OccupancyGridConverterNode::tryWriteMapToFile(
        const nav_msgs::msg::OccupancyGrid &map,
        const SaveParameters &save_parameters)
    {
        std::cout << "[INFO] [tryWriteMapToFile]: Received a " << map.info.width << " X " << map.info.height << " map @ " << map.info.resolution << " m/pix" << std::endl;

        std::string mapdatafile = save_parameters.map_file_name + "." + save_parameters.image_format;
        {
            Magick::Image image({map.info.width, map.info.height}, "red");

            image.type(
                save_parameters.mode == nav2_map_server::MapMode::Scale ? Magick::TrueColorMatteType : Magick::GrayscaleType);
            image.depth(8);

            int free_thresh_int = std::rint(save_parameters.free_thresh * 100.0);
            int occupied_thresh_int = std::rint(save_parameters.occupied_thresh * 100.0);

            for (size_t y = 0; y < map.info.height; y++)
            {
                for (size_t x = 0; x < map.info.width; x++)
                {
                    int8_t map_cell = map.data[map.info.width * (map.info.height - y - 1) + x];

                    Magick::Color pixel;

                    switch (save_parameters.mode)
                    {
                    case nav2_map_server::MapMode::Trinary:
                        if (map_cell < 0 || 100 < map_cell)
                        {
                            pixel = Magick::ColorGray(205 / 255.0);
                        }
                        else if (map_cell <= free_thresh_int)
                        {
                            pixel = Magick::ColorGray(254 / 255.0);
                        }
                        else if (occupied_thresh_int <= map_cell)
                        {
                            pixel = Magick::ColorGray(0 / 255.0);
                        }
                        else
                        {
                            pixel = Magick::ColorGray(205 / 255.0);
                        }
                        break;
                    case nav2_map_server::MapMode::Scale:
                        if (map_cell < 0 || 100 < map_cell)
                        {
                            pixel = Magick::ColorGray{0.5};
                            pixel.alphaQuantum(TransparentOpacity);
                        }
                        else
                        {
                            pixel = Magick::ColorGray{(100.0 - map_cell) / 100.0};
                        }
                        break;
                    case nav2_map_server::MapMode::Raw:
                        Magick::Quantum q;
                        if (map_cell < 0 || 100 < map_cell)
                        {
                            q = MaxRGB;
                        }
                        else
                        {
                            q = map_cell / 255.0 * MaxRGB;
                        }
                        pixel = Magick::Color(q, q, q);
                        break;
                    default:
                        std::cerr << "[ERROR] [tryWriteMapToFile]: Map mode should be Trinary, Scale or Raw" << std::endl;
                        throw std::runtime_error("Invalid map mode");
                    }
                    image.pixelColor(x, y, pixel);
                }
            }

            std::cout << "[INFO] [tryWriteMapToFile]: Writing map occupancy data to " << mapdatafile << std::endl;
            image.write(mapdatafile);
        }

        std::string mapmetadatafile = save_parameters.map_file_name + ".yaml";
        {
            std::ofstream yaml(mapmetadatafile);

            geometry_msgs::msg::Quaternion orientation = map.info.origin.orientation;
            tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
            double yaw, pitch, roll;
            mat.getEulerYPR(yaw, pitch, roll);

            YAML::Emitter e;
            e << YAML::Precision(3);
            e << YAML::BeginMap;
            e << YAML::Key << "image" << YAML::Value << mapdatafile;
            e << YAML::Key << "mode" << YAML::Value << map_mode_to_string(save_parameters.mode);
            e << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
            e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << map.info.origin.position.x << map.info.origin.position.y << yaw << YAML::EndSeq;
            e << YAML::Key << "negate" << YAML::Value << 0;
            e << YAML::Key << "occupied_thresh" << YAML::Value << save_parameters.occupied_thresh;
            e << YAML::Key << "free_thresh" << YAML::Value << save_parameters.free_thresh;

            if (!e.good())
            {
                std::cout << "[WARN] [tryWriteMapToFile]: YAML writer failed with an error " << e.GetLastError() << ". The map metadata may be invalid." << std::endl;
            }

            std::cout << "[INFO] [tryWriteMapToFile]: Writing map metadata to " << mapmetadatafile << std::endl;
            std::ofstream(mapmetadatafile) << e.c_str();
        }
        std::cout << "[INFO] [tryWriteMapToFile]: Map saved" << std::endl;
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
