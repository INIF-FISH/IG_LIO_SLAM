
/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * Only this file in under MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 23:19
 * Description: Config header
 */


#pragma once

#include <iostream>
#include <string>
#include <pcl/point_types.h>

#include "octomap/octomap.h"
#define ANSI_RED "\033[1m\x1b[31m"
#define ANSI_GREEN "\033[1m\x1b[32m"
#define ANSI_YELLOW "\033[1m\x1b[33m"
#define ANSI_BLUE "\033[1m\x1b[34m"
#define ANSI_MAGENTA "\033[1m\x1b[35m"
#define ANSI_CYAN "\033[1m\x1b[36m"
#define ANSI_RESET "\x1b[0m"
#define ANSI_BOLD "\033[1m"

// CHANGE Point Type Here!!! If you want to use XYZI, change to pcl::PointXYZI
// typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZ PointType;
typedef octomap::OcTree OcTreeT;

// typedef pcl::PointXYZRGB PointType;
// typedef octomap::ColorOcTree OcTreeT;

namespace common {
struct Config {

  // octomap params, default from octomap_mapping repo
  float m_res = 0.1;  // resolution of the map
  float probHit = 0.7;
  float probMiss = 0.4;
  float thresMin = 0.12;
  float thresMax = 0.97;
  bool m_prune = true;  // prune the tree after insertion of new data

  // range, // -1 means no range limit
  float m_maxRange = -1.0;
  float m_minRange = -1.0;


  bool verbose_ = false;  // print out logs
  bool replace_intensity = false;

  // SAC segmentation params ground plane
  bool filterGroundPlane = false;
  float m_groundFilterDistance = 0.04;
  float m_groundFilterAngle = 0.15;
  float m_groundFilterPlaneDistance = 0.07;

  // filter noise
  bool filterNoise = false;
  float StddevMulThresh = 1.0;
  int filterMeanK = 50;
};

}  // namespace common