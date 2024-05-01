#include "../include/occupancy_grid_converter.h"

namespace IG_LIO
{
    OccupancyGridConverterNode::OccupancyGridConverterNode(const rclcpp::NodeOptions &options)
        : Node("occupancy_grid_converter", options), filterChain_local_("grid_map::GridMap"), filterChain_map_("grid_map::GridMap")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), GREEN << "Starting occupancy_grid_converter node ..." << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), BLUE << "processing params ..." << RESET);
        param_respond();
        RCLCPP_INFO_STREAM(this->get_logger(), BLUE << "processing subscribers ..." << RESET);
        initSubscribers();
        RCLCPP_INFO_STREAM(this->get_logger(), BLUE << "processing publishers ..." << RESET);
        initPublishers();
        RCLCPP_INFO_STREAM(this->get_logger(), BLUE << "processing serivces ..." << RESET);
        initSerivces();
        RCLCPP_INFO_STREAM(this->get_logger(), BLUE << "processing init ..." << RESET);
        init();
        RCLCPP_INFO_STREAM(this->get_logger(), GREEN << "DONE." << RESET);
    }

    OccupancyGridConverterNode::~OccupancyGridConverterNode()
    {
    }

    void OccupancyGridConverterNode::param_respond()
    {
        this->declare_parameter<std::string>("map_frame", std::string("map"));
        this->declare_parameter<std::string>("local_frame", std::string("local"));
        this->declare_parameter<std::string>("robot_frame", std::string("base_link"));
        this->declare_parameter<int>("grid_map_cloud_size", 10);
        this->declare_parameter<double>("occupancyGriddataMin", 0.65);
        this->declare_parameter<double>("occupancyGriddataMax", 10.0);
        this->declare_parameter<double>("point_min_height", 0.0);
        this->declare_parameter<double>("point_max_height", 10.0);
        this->declare_parameter<double>("min_distance", 0.4);
        this->declare_parameter("filter_chain_parameter_name_local", std::string("filters_local"));
        this->declare_parameter("filter_chain_parameter_name_map", std::string("filters_map"));
        this->get_parameter("map_frame", map_frame);
        this->get_parameter("local_frame", local_frame);
        this->get_parameter("robot_frame", robot_frame);
        this->get_parameter("grid_map_cloud_size", grid_map_cloud_size);
        this->get_parameter("occupancyGriddataMin", occupancyGriddataMin);
        this->get_parameter("occupancyGriddataMax", occupancyGriddataMax);
        this->get_parameter("point_min_height", point_min_height);
        this->get_parameter("point_max_height", point_max_height);
        this->get_parameter("min_distance", min_distance);
        this->get_parameter("filter_chain_parameter_name_local", filterChainParametersName_local_);
        this->get_parameter("filter_chain_parameter_name_map", filterChainParametersName_map_);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "map_frame set to " << map_frame << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "local_frame set to " << local_frame << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "robot_frame set to " << robot_frame << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "grid_map_cloud_size set to " << grid_map_cloud_size << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "occupancyGriddataMin set to " << occupancyGriddataMin << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "occupancyGriddataMax set to " << occupancyGriddataMax << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "point_min_height set to " << point_min_height << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "point_max_height set to " << point_max_height << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "min_distance set to " << min_distance << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "filter_chain_parameter_name_local set to " << filterChainParametersName_local_ << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "filter_chain_parameter_name_map set to " << filterChainParametersName_map_ << RESET);
    }

    void OccupancyGridConverterNode::initSubscribers()
    {
        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "local_cloud", rclcpp::QoS(10).transient_local().keep_last(1), std::bind(&OccupancyGridConverterNode::pointCloudCallback, this, std::placeholders::_1));
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "point cloud data subscribe from "
                                                    << "local_cloud" << RESET);
    }

    void OccupancyGridConverterNode::initPublishers()
    {
        local_grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
            "/local_costmap/grid_map", rclcpp::QoS(10).transient_local().keep_last(1));
        occupancy_grid_pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(10).reliable().transient_local().keep_last(1));
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "local grid map publish to "
                                                    << "/local_costmap/grid_map" << RESET);
        RCLCPP_INFO_STREAM(this->get_logger(), CYAN << "occupancy grid map publish to "
                                                    << "map" << RESET);
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
            RCLCPP_INFO_STREAM(this->get_logger(), GREEN << "Filter chain local configured." << RESET);
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), RED << "Could not configure the filter chain local!" << RESET);
            rclcpp::shutdown();
            return;
        }
        if (filterChain_map_.configure(
                filterChainParametersName_map_, this->get_node_logging_interface(),
                this->get_node_parameters_interface()))
        {
            RCLCPP_INFO_STREAM(this->get_logger(), GREEN << "Filter chain map configured." << RESET);
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), RED << "Could not configure the filter chain map!" << RESET);
            rclcpp::shutdown();
            return;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr OccupancyGridConverterNode::filterPointCloudByHeightRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // 创建PassThrough滤波器
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(input_cloud);
        pass.setFilterFieldName("z");                             // 设置过滤字段为z轴
        pass.setFilterLimits(point_min_height, point_max_height); // 设置高度范围
        pass.filter(*cloud_filtered);                             // 应用过滤器

        return cloud_filtered;
    }

    grid_map::GridMap OccupancyGridConverterNode::makeGridMapFromDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr, builtin_interfaces::msg::Time stamp)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        geometry_msgs::msg::TransformStamped tf_map_to_base;
        try
        {
            tf_map_to_base = tfBuffer_->lookupTransform(map_frame, robot_frame, stamp, rclcpp::Duration::from_seconds(0.1));
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
        if (grid_map_cloud_.size() == std::size_t(grid_map_cloud_size))
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

        gridMap.setFrameId(local_frame);
        gridMap.setTimestamp(this->get_clock()->now().nanoseconds());
        grid_map::GridMap outputMap;
        if (!filterChain_local_.update(gridMap, outputMap))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), RED << "Could not update the grid map filter chain local!" << RESET);
            return gridMap;
        }
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
        gridMap.setFrameId(map_frame);
        gridMap.setTimestamp(this->get_clock()->now().nanoseconds());
        grid_map::GridMap outputMap;
        if (!filterChain_map_.update(gridMap, outputMap))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), RED << "Could not update the grid map filter chain map!" << RESET);
            return gridMap;
        }
        return outputMap;
    }

    void OccupancyGridConverterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), MAGENTA << "Received PointCloud2 message. Height: " << msg->height << ", Width: " << msg->width << RESET);
        pcl::PointCloud<pcl::PointXYZ> in_cloud;
        pcl::fromROSMsg(*msg, in_cloud);
        pcl::shared_ptr<grid_map::grid_map_pcl::Pointcloud> cloud_ptr = in_cloud.makeShared();
        auto cloud_filtered = filterPointCloudByHeightRange(cloud_ptr);
        if (cloud_filtered->points.size() < 100)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), RED << "Couldn't find enough fit points !" << RESET);
            return;
        }
        auto gridMap = makeGridMapFromDepth(cloud_filtered, msg->header.stamp);
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
            this->occupancy_grid_pub_map_->publish(std::move(*occupancyGrid_to_pub));
        }
    }

    void OccupancyGridConverterNode::covertMapCallBack(const ig_lio_c_msgs::srv::CovertMap::Request::SharedPtr request,
                                                       const ig_lio_c_msgs::srv::CovertMap::Response::SharedPtr response)
    {
        Magick::InitializeMagick(nullptr);
        if (request->pcd_path.empty())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), RED << "PCD file_path empty !" << RESET);
            response->status = 0;
            response->message = "Failed to read PCD file";
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(request->pcd_path, *cloud) == -1)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), RED << "Couldn't read PCD file" << RESET);
            response->status = 0;
            response->message = "Failed to read PCD file";
            return;
        }
        auto cloud_filtered = filterPointCloudByHeightRange(cloud);
        if (cloud_filtered->points.size() < 100)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), RED << "Couldn't find enough fit points !" << RESET);
            response->status = 0;
            response->message = "Couldn't find fit points !";
            return;
        }
        auto gridMap = makeGridMap(cloud_filtered);
        auto occ_grid = createOccupancyGridMsg(gridMap);
        occ_grid->header.frame_id = map_frame;
        SaveParameters save_parameters;
        {
            save_parameters.image_format = request->image_format;
        }
        if (request->map_file_name.empty())
        {
            RCLCPP_WARN_STREAM(this->get_logger(), YELLOW << "Non map_file_name, using map." << RESET);
            save_parameters.map_file_name = "map";
        }
        else
        {
            save_parameters.map_file_name = request->map_file_name;
        }
        if (request->occupied_thresh == 0.0)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), YELLOW << "Non occupied_thresh, using 0.65." << RESET);
            save_parameters.occupied_thresh = 0.65;
        }
        else
        {
            save_parameters.occupied_thresh = request->occupied_thresh;
        }
        if (request->free_thresh == 0.0)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), YELLOW << "Non free_thresh, using 0.25." << RESET);
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
            RCLCPP_WARN_STREAM(this->get_logger(), YELLOW << "Map mode parameter not recognized: " << request->map_mode.c_str() << ", using default value (trinary)" << RESET);
        }
        if (save_parameters.image_format == "")
        {
            save_parameters.image_format = save_parameters.mode == nav2_map_server::MapMode::Scale ? "png" : "pgm";
            RCLCPP_WARN_STREAM(this->get_logger(), YELLOW << "[WARN] [covertMapCallBack]: Image format unspecified. Setting it to: " << save_parameters.image_format << RESET);
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
            RCLCPP_WARN_STREAM(this->get_logger(), YELLOW << "[WARN] [covertMapCallBack]: Requested image format '" << save_parameters.image_format << "' is not one of the recommended formats: " << ss.str() << RESET);
        }
        const std::string FALLBACK_FORMAT = "png";

        try
        {
            Magick::CoderInfo info(save_parameters.image_format);
            if (!info.isWritable())
            {
                RCLCPP_WARN_STREAM(this->get_logger(), YELLOW << "[WARN] [covertMapCallBack]: Format '" << save_parameters.image_format << "' is not writable. Using '" << FALLBACK_FORMAT << "' instead" << RESET);
                save_parameters.image_format = FALLBACK_FORMAT;
            }
        }
        catch (Magick::ErrorOption &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), RED << "[ERROR] [covertMapCallBack]: ERR " << e.what() << RESET);
            save_parameters.image_format = FALLBACK_FORMAT;
        }

        if (
            save_parameters.mode == nav2_map_server::MapMode::Scale &&
            (save_parameters.image_format == "pgm" ||
             save_parameters.image_format == "jpg" ||
             save_parameters.image_format == "jpeg"))
        {
            RCLCPP_WARN_STREAM(this->get_logger(), YELLOW << "[WARN] [covertMapCallBack]: Map mode 'scale' requires transparency, but format '" << save_parameters.image_format << "' does not support it. Consider switching image format to 'png'." << RESET);
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
        RCLCPP_INFO_STREAM(this->get_logger(), MAGENTA << "[INFO] [tryWriteMapToFile]: Received a " << map.info.width << " X " << map.info.height << " map @ " << map.info.resolution << " m/pix" << RESET);

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
                        RCLCPP_ERROR_STREAM(this->get_logger(), RED << "[ERROR] [tryWriteMapToFile]: Map mode should be Trinary, Scale or Raw" << RESET);
                        throw std::runtime_error("Invalid map mode");
                    }
                    image.pixelColor(x, y, pixel);
                }
            }

            RCLCPP_INFO_STREAM(this->get_logger(), MAGENTA << "[INFO] [tryWriteMapToFile]: Writing map occupancy data to " << mapdatafile << RESET);
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
                RCLCPP_WARN_STREAM(this->get_logger(), YELLOW << "[WARN] [tryWriteMapToFile]: YAML writer failed with an error " << e.GetLastError() << ". The map metadata may be invalid." << RESET);
            }

            RCLCPP_INFO_STREAM(this->get_logger(), MAGENTA << "[INFO] [tryWriteMapToFile]: Writing map metadata to " << mapmetadatafile << RESET);
            std::ofstream(mapmetadatafile) << e.c_str();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), MAGENTA << "[INFO] [tryWriteMapToFile]: Map saved" << RESET);
    }
} // namespace IG_LIO

bool terminate_flag = false;

void signalHandler(int signum)
{
    switch (signum)
    {
    case SIGALRM:
        break;
    case SIGINT:
        std::cout << RED << "SHUTTING DOWN CONVERTER NODE!" << RESET << std::endl;
        terminate_flag = true;
        break;
    default:
        break;
    }
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
