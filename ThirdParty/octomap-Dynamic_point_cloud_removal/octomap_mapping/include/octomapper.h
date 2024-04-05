/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-04-07 13:30
 * @details: No ROS version, speed up the process, check our benchmark in dufomap
 */
#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "octomap/octomap.h"
#include "utils.h"
#include "timing.hpp"
namespace octomap {
class MapUpdater {
public:
  MapUpdater(const std::string &config_file_path);
  virtual ~MapUpdater() = default;
  void setConfig();
  void run(pcl::PointCloud<PointType>::Ptr const& single_pc);
  void saveMap(std::string const& folder_path, std::string const& file_name);
  void saveRawMap(std::string const& folder_path, std::string const& file_name);
  pcl::PointCloud<PointType>::Ptr getRawMap();
  const common::Config getCfg() { return cfg_; }
  ufo::Timing timing;


private:
    YAML::Node yconfig;
    common::Config cfg_;
    OcTreeT* m_octree;
    octomap::KeyRay m_keyRay;  // temp storage for ray casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;
    pcl::PointCloud<PointType>::Ptr ground_pts;
    pcl::PointCloud<PointType>::Ptr raw_map_ptr_;
    void filterGroundPlane(pcl::PointCloud<PointType>::Ptr const& pc, 
                           pcl::PointCloud<PointType>::Ptr &ground, 
                           pcl::PointCloud<PointType>::Ptr &nonground);
    pcl::PointCloud<PointType>::Ptr noise_cloud;

protected:
    inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
      for (unsigned i = 0; i < 3; ++i)
        min[i] = std::min(in[i], min[i]);
    };

    inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
      for (unsigned i = 0; i < 3; ++i)
        max[i] = std::max(in[i], max[i]);
    };

};

}  // namespace octomap