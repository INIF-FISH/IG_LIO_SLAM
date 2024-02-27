#include "../../../include/ig_lio_c/voxel_map/voxel_map.h"

namespace IG_LIO
{
    bool compare(std::pair<Eigen::Vector3d, double> &p1, std::pair<Eigen::Vector3d, double> &p2)
    {
        return p1.second < p2.second;
    }

    size_t HashUtil::operator()(const Eigen::Vector3d &point)
    {
        double loc_xyz[3];
        for (size_t i = 0; i < 3; ++i)
        {
            loc_xyz[i] = point[i] * resolution_inv_;
            if (loc_xyz[i] < 0)
            {
                loc_xyz[i] -= 1.0;
            }
        }
        size_t x = static_cast<size_t>(loc_xyz[0]);
        size_t y = static_cast<size_t>(loc_xyz[1]);
        size_t z = static_cast<size_t>(loc_xyz[2]);

        return ((((z)*hash_p_) % max_n_ + (y)) * hash_p_) % max_n_ + (x);
    }

    void Grid::setMinMax(size_t min_n, size_t max_n)
    {
        min_num = min_n;
        max_num = max_n;
    }

    void Grid::updateConv()
    {
        if (is_updated)
            return;
        is_updated = true;
        if (points_num < min_num)
        {
            is_valid = false;
            return;
        }
        conv = (conv_sum - points_sum * centroid.transpose()) / (static_cast<double>(points_num) - 1.0);
        is_valid = true;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(conv, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector3d val(1, 1, 1e-3);
        conv = svd.matrixU() * val.asDiagonal() * svd.matrixV().transpose();
    }

    void Grid::addPoint(Eigen::Vector3d &point, bool insert)
    {
        is_updated = false;
        points_num++;
        points_sum += point;
        centroid = points_sum / static_cast<double>(points_num);
        conv_sum += (point * point.transpose());
        if (points.size() >= max_num * 2 || !insert)
            return;
        points.push_back(point);
    }

    void VoxelMap::initializeDelta()
    {
        delta_.clear();
        switch (mode_)
        {
        case MODE::NEARBY_1:
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            break;
        case MODE::NEARBY_7:
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
            break;
        case MODE::NEARBY_19:
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, -resolution_));
            break;
        default:
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, -resolution_, 0.0));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, 0.0, -resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(0.0, -resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, -resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(resolution_, -resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, resolution_, -resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, -resolution_, resolution_));
            delta_.push_back(Eigen::Vector3d(-resolution_, -resolution_, -resolution_));
            break;
        }
    }

    VoxelMap::VoxelMap(double resolution, size_t capacity, size_t grid_capacity, MODE mode)
        : resolution_(resolution),
          capacity_(capacity),
          grid_capacity_(grid_capacity),
          mode_(mode),
          hash_util_(resolution)
    {
        initializeDelta();
    }

    bool VoxelMap::isSameGrid(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
    {
        int hx_1 = floor(p1.x() * resolution_inv_);
        int hy_1 = floor(p1.y() * resolution_inv_);
        int hz_1 = floor(p1.z() * resolution_inv_);
        int hx_2 = floor(p2.x() * resolution_inv_);
        int hy_2 = floor(p2.y() * resolution_inv_);
        int hz_2 = floor(p2.z() * resolution_inv_);

        return ((hx_1 == hx_2) && (hy_1 == hy_2) && (hz_1 == hz_2));
    }

    size_t VoxelMap::addCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        size_t error_count = 0;
        if (cloud->empty())
            return 0;
        current_idx_.clear();
        for (pcl::PointXYZINormal &point : cloud->points)
        {
            Eigen::Vector3d point_vec(point.x, point.y, point.z);
            size_t idx = hash_util_(point_vec);
            current_idx_.push_back(idx);
            auto iter = storage_.find(idx);
            if (iter == storage_.end())
            {
                std::shared_ptr<Grid> grid = std::make_shared<Grid>(grid_capacity_);
                grid->addPoint(point_vec, true);
                cache_.push_front(idx);
                storage_.insert({idx, {cache_.begin(), grid}});
                if (storage_.size() >= capacity_)
                {
                    storage_.erase(cache_.back());
                    cache_.pop_back();
                }
            }
            else
            {
                if (!isSameGrid(iter->second.second->centroid, point_vec))
                {
                    error_count++;
                    cache_.erase(iter->second.first);
                    std::shared_ptr<Grid> grid = std::make_shared<Grid>(grid_capacity_);
                    grid->addPoint(point_vec, true);
                    cache_.push_front(idx);
                    storage_[idx].first = cache_.begin();
                    storage_[idx].second = grid;
                }
                else
                {
                    size_t point_in_grid = iter->second.second->points_num;
                    if (point_in_grid < 50)
                        iter->second.second->addPoint(point_vec, point_in_grid < grid_capacity_ ? true : false);
                    cache_.splice(cache_.begin(), cache_, iter->second.first);
                    storage_[idx].first = cache_.begin();
                }
            }
        }
        for (size_t &idx : current_idx_)
        {
            storage_[idx].second->updateConv();
        }
        return error_count;
    }

    void VoxelMap::reset()
    {
        cache_.clear();
        storage_.clear();
    }

    bool VoxelMap::searchKNN(const Eigen::Vector3d &point, size_t K, double range, std::vector<Eigen::Vector3d> &results)
    {
        tbb::concurrent_vector<std::pair<Eigen::Vector3d, double>> candidates;
        double range2 = range * range;

        tbb::parallel_for(tbb::blocked_range<size_t>(0, delta_.size()),
                          [&](const tbb::blocked_range<size_t> &r)
                          {
                              std::vector<std::pair<Eigen::Vector3d, double>> local_candidates;
                              local_candidates.reserve(r.size());
                              for (size_t i = r.begin(); i != r.end(); ++i)
                              {
                                  const Eigen::Vector3d &delta = delta_[i];
                                  Eigen::Vector3d near_by = point + delta;
                                  size_t hash_idx = hash_util_(near_by);
                                  auto iter = storage_.find(hash_idx);
                                  if (iter != storage_.end() && isSameGrid(near_by, iter->second.second->centroid))
                                  {
                                      for (Eigen::Vector3d &p : iter->second.second->points)
                                      {
                                          double dist = (point - p).squaredNorm();
                                          if (dist < range2)
                                              local_candidates.emplace_back(p, dist);
                                      }
                                  }
                              }
                              auto it = candidates.grow_by(local_candidates.size());
                              std::copy(local_candidates.begin(), local_candidates.end(), it);
                          });

        if (candidates.empty())
            return false;

        tbb::parallel_sort(candidates.begin(), candidates.end(), compare);
        if (candidates.size() > K)
            candidates.resize(K);

        results.clear();

        for (const auto &it : candidates)
        {
            results.emplace_back(it.first);
        }

        return true;
    }

    bool VoxelMap::getCentroidAndCovariance(size_t hash_idx, Eigen::Vector3d &centroid, Eigen::Matrix3d &cov)
    {
        auto iter = storage_.find(hash_idx);
        if (iter != storage_.end() && iter->second.second->is_valid)
        {
            centroid = iter->second.second->centroid;
            cov = iter->second.second->conv;
            return true;
        }
        return false;
    }

    bool VoxelMap::getCentroidAndCovariance(const Eigen::Vector3d &point, Eigen::Vector3d &centroid, Eigen::Matrix3d &cov)
    {
        size_t idx = hash_util_(point);
        return getCentroidAndCovariance(idx, centroid, cov);
    }

    void FastGrid::addPoint(const Eigen::Vector3d &point)
    {
        num += 1;
        centroid += ((point - centroid) / static_cast<double>(num));
    }

    FastVoxelMap::FastVoxelMap(double resolution) : resolution_(resolution), hash_util_(resolution)
    {
        search_range_.clear();
        for (int x_gain = -1; x_gain <= 1; ++x_gain)
        {
            for (int y_gain = -1; y_gain <= 1; ++y_gain)
            {
                for (int z_gain = -1; z_gain <= 1; ++z_gain)
                {
                    search_range_.emplace_back(Eigen::Vector3d(x_gain * resolution_,
                                                               y_gain * resolution_,
                                                               z_gain * resolution_));
                }
            }
        }
    }

    void FastVoxelMap::filter(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, std::vector<PointCov> &outs)
    {
        if (cloud->empty())
            return;
        std::mutex mtx;
        outs.clear();
        grid_array_.clear();
        grid_map_.clear();
        grid_array_.reserve(cloud->size());

        tbb::concurrent_unordered_map<size_t, std::shared_ptr<FastGrid>> concurrent_grid_map;

        tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud->size()), [&](const tbb::blocked_range<size_t> &range)
                          {
                              std::vector<std::shared_ptr<IG_LIO::FastGrid>> local_grid_array_;
                              local_grid_array_.reserve(range.size());
                              for (size_t i = range.begin(); i != range.end(); ++i)
                              {
                                  pcl::PointXYZINormal &p = cloud->points[i];
                                  Eigen::Vector3d p_vec(p.x, p.y, p.z);
                                  size_t idx = hash_util_(p_vec);
                                  auto iter = concurrent_grid_map.find(idx);

                                  if (iter == concurrent_grid_map.end())
                                  {
                                      std::shared_ptr<FastGrid> grid = std::make_shared<FastGrid>();
                                      grid->addPoint(p_vec);

                                      concurrent_grid_map.insert(concurrent_grid_map.end(), {idx, grid});
                                      local_grid_array_.emplace_back(grid);
                                  }
                                  else
                                  {
                                      iter->second->addPoint(p_vec);
                                  }
                              }
                              std::lock_guard<std::mutex> lck(mtx);
                              std::copy(local_grid_array_.begin(), local_grid_array_.end(), std::back_inserter(grid_array_)); });

        tbb::parallel_for(tbb::blocked_range<size_t>(0, grid_array_.size()), [&](const tbb::blocked_range<size_t> &range)
                          {
        std::vector<PointCov> local_outs;
        local_outs.reserve(range.size());
        for (size_t i = range.begin(); i != range.end(); ++i)
        {
            std::shared_ptr<FastGrid> &g = grid_array_[i];

            Eigen::Vector3d points_sum = Eigen::Vector3d::Zero();
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            size_t points_num = 0;

            for (Eigen::Vector3d &r : search_range_)
            {
                Eigen::Vector3d near = g->centroid + r;
                auto iter = concurrent_grid_map.find(hash_util_(near));
                if (iter != concurrent_grid_map.end())
                {
                    points_sum += iter->second->centroid;
                    cov += iter->second->centroid * iter->second->centroid.transpose();
                    points_num += 1;
                }
            }

            if (points_num >= 6)
            {
                Eigen::Vector3d centriod = points_sum / static_cast<double>(points_num);
                cov = (cov - points_sum * centriod.transpose()) / (static_cast<double>(points_num) - 1.0);
                Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::Vector3d values(1, 1, 1e-3);
                cov = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
            }
            local_outs.emplace_back(g->centroid, cov);
        }
        std::lock_guard<std::mutex> lck(mtx);
        std::copy(local_outs.begin(), local_outs.end(), std::back_inserter(outs)); });
    }
} // namespace IG_LIO
