/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-04-05 23:28
 * @details: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map
 */

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "octomapper.h"
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_colorlogtostderr = true;
    google::SetStderrLogging(google::INFO);

    if (argc < 3) {
        LOG(ERROR) << "Usage: " << argv[0] << " [pcd_folder] [config_file]";
        return 0;
    }
    std::string pcd_parent = argv[1]; // we assume that rawmap is in pcd_parent;
    std::string config_file = argv[2];
    int cnt = 1, run_max = 1;
    // check if the config_file exists
    if (!std::filesystem::exists(config_file)) {
        LOG(ERROR) << "Config file does not exist: " << config_file;
        return 0;
    }

    octomap::MapUpdater map_updater(config_file);

    std::vector<std::string> filenames;
    for (const auto &entry : std::filesystem::directory_iterator(std::filesystem::path(pcd_parent) / "pcd")) {
        filenames.push_back(entry.path().string());
    }

    // sort the filenames
    std::sort(filenames.begin(), filenames.end());
    int total = filenames.size();
    if (argc > 3) {
        run_max = std::stoi(argv[3]);
        if (run_max == -1) {
            LOG(INFO) << "We will run all the frame in sequence, the total "
                         "number is: "
                      << total;
            run_max = total + 1;
        }
    }

    for (const auto & filename : filenames) {
        map_updater.timing.start(" One Scan Cost  ");
        if(cnt>1 && !map_updater.getCfg().verbose_){
            std::ostringstream log_msg;
            log_msg << "(" << cnt << "/" << run_max << ") Processing: " << filename << " Time Cost: " 
                << map_updater.timing.lastSeconds(" One Scan Cost  ") << "s";
            std::string spaces(10, ' ');
            log_msg << spaces;
            // std::cout <<log_msg.str()<<std::endl;
            std::cout << "\r" <<log_msg.str() << std::flush;
        }

        if (filename.substr(filename.size() - 4) != ".pcd")
            continue;

        pcl::PointCloud<PointType>::Ptr pcd(new pcl::PointCloud<PointType>);
        pcl::io::loadPCDFile<PointType>(filename, *pcd);
        map_updater.run(pcd);
        map_updater.timing.stop(" One Scan Cost  ");
        cnt++;
        if(cnt>run_max)
            break;
    }
    map_updater.timing.start("4. Query & Write");
    
    std::string output_file;
    if(map_updater.getCfg().filterGroundPlane && map_updater.getCfg().filterNoise)
        output_file = "octomapfg";
    else if(map_updater.getCfg().filterGroundPlane)
        output_file = "octomapg";
    else
        output_file = "octomap";

    // map_updater.saveMap(pcd_parent, output_file); // query the center point in octree directly
    map_updater.saveRawMap(pcd_parent, output_file);
    
    map_updater.timing.stop("4. Query & Write");

    // set print color
	map_updater.timing.setColor("0. Fit ground   ", ufo::Timing::boldYellowColor());
	map_updater.timing.setColor("1. Ray SetFreeOc", ufo::Timing::boldCyanColor());
	map_updater.timing.setColor("2. Update Octree", ufo::Timing::boldMagentaColor());
	map_updater.timing.setColor("3. Prune Tree   ", ufo::Timing::boldGreenColor());
	map_updater.timing.setColor("4. Query & Write", ufo::Timing::boldRedColor());
	// timing.setColor("5. Write     ", ufo::Timing::boldBlueColor());

	printf("\nOctomap Timings:\n");
	printf("\t Component\t\tTotal\tLast\tMean\tStDev\t Min\t Max\t Steps\n");
	for (auto const& tag : map_updater.timing.tags()) {
		printf("\t%s%s\t%5.2f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%6lu%s\n",
		       map_updater.timing.color(tag).c_str(), tag.c_str(), map_updater.timing.totalSeconds(tag),
		       map_updater.timing.lastSeconds(tag), map_updater.timing.meanSeconds(tag), map_updater.timing.stdSeconds(tag),
		       map_updater.timing.minSeconds(tag), map_updater.timing.maxSeconds(tag), map_updater.timing.numSamples(tag),
		       ufo::Timing::resetColor());
	}
    LOG(INFO) << ANSI_GREEN << "Done! " << ANSI_RESET << "Check the output in " << pcd_parent << ", file: " << output_file + "_output.pcd";
    return 0;
}
