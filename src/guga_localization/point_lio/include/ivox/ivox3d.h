//
// Created by xiang on 2021/9/16.
//

#ifndef FASTER_LIO_IVOX3D_H
#define FASTER_LIO_IVOX3D_H

#include <list>
#include <memory>
#include <unordered_map>

#include "eigen_types.h"
#include "ivox3d_node.hpp"

namespace faster_lio {

  enum class IVoxNodeType {
    DEFAULT,  // linear ivox
    PHC,      // phc ivox
  };

  /// traits for NodeType
  template <IVoxNodeType node_type, typename PointT, int dim>
  struct IVoxNodeTypeTraits {};

  template <typename PointT, int dim>
  struct IVoxNodeTypeTraits<IVoxNodeType::DEFAULT, PointT, dim> {
    using NodeType = IVoxNode<PointT, dim>;
  };

  template <typename PointT, int dim>
  struct IVoxNodeTypeTraits<IVoxNodeType::PHC, PointT, dim> {
    using NodeType = IVoxNodePhc<PointT, dim>;
  };

  template <int dim = 3, IVoxNodeType node_type = IVoxNodeType::DEFAULT,
            typename PointType = pcl::PointXYZ>
  class IVox {
  public:
    using KeyType = Eigen::Matrix<int, dim, 1>;
    using PtType = Eigen::Matrix<float, dim, 1>;
    using NodeType =
        typename IVoxNodeTypeTraits<node_type, PointType, dim>::NodeType;
    using PointVector =
        std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using DistPoint = typename NodeType::DistPoint;
    using Ptr = std::shared_ptr<IVox<dim, node_type, PointType>>;

    enum class NearbyType {
      CENTER,  // center only
      NEARBY6,
      NEARBY18,
      NEARBY26,
    };

    struct Options {
      float resolution_ = 0.2;                        // ivox resolution
      float inv_resolution_ = 10.0;                   // inverse resolution
      NearbyType nearby_type_ = NearbyType::NEARBY6;  // nearby range
      std::size_t capacity_ = 1000000;                // capacity
    };

    /**
     * constructor
     * @param options  ivox options
     */
    explicit IVox(Options options) : options_(options) {
      options_.inv_resolution_ = 1.0 / options_.resolution_;
      GenerateNearbyGrids();
    }

    /**
     * add points
     * @param points_to_add
     */
    void AddPoints(const PointVector& points_to_add);

    /// get nn with condition
    bool GetClosestPoint(const PointType& pt, PointVector& closest_pt,
                         int max_num = 5, double max_range = 5.0);

  private:
    /// generate the nearby grids according to the given options
    void GenerateNearbyGrids();

    KeyType Pos2Grid(const PtType& pt) const;

    Options options_;
    std::unordered_map<
        KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator,
        hash_vec<dim>>
        grids_map_;
    std::list<std::pair<KeyType, NodeType>> grids_cache_;  // voxel cache
    std::vector<KeyType> nearby_grids_;                    // nearbys
  };

  template <int dim, IVoxNodeType node_type, typename PointType>
  bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt,
                                                        PointVector& closest_pt,
                                                        int max_num,
                                                        double max_range) {
    std::vector<DistPoint> candidates;
    candidates.reserve(max_num * nearby_grids_.size());

    auto key = Pos2Grid(ToEigen<float, dim>(pt));

    for (const KeyType& delta : nearby_grids_) {
      auto dkey = key + delta;
      auto iter = grids_map_.find(dkey);
      if (iter != grids_map_.end()) {
        iter->second->second.KNNPointByCondition(candidates, pt, max_num,
                                                 max_range);
      }
    }

    if (candidates.empty()) {
      return false;
    }

    if ((int)candidates.size() <= max_num) {
    } else {
      std::nth_element(candidates.begin(), candidates.begin() + max_num - 1,
                       candidates.end());
      candidates.resize(max_num);
    }
    std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

    closest_pt.clear();
    for (auto& it : candidates) {
      closest_pt.emplace_back(it.Get());
    }
    return closest_pt.empty() == false;
  }

  template <int dim, IVoxNodeType node_type, typename PointType>
  void IVox<dim, node_type, PointType>::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
      nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
      nearby_grids_ = {KeyType(0, 0, 0), KeyType(-1, 0, 0), KeyType(1, 0, 0),
                       KeyType(0, 1, 0), KeyType(0, -1, 0), KeyType(0, 0, -1),
                       KeyType(0, 0, 1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
      nearby_grids_ = {
          KeyType(0, 0, 0),  KeyType(-1, 0, 0),  KeyType(1, 0, 0),
          KeyType(0, 1, 0),  KeyType(0, -1, 0),  KeyType(0, 0, -1),
          KeyType(0, 0, 1),  KeyType(1, 1, 0),   KeyType(-1, 1, 0),
          KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
          KeyType(-1, 0, 1), KeyType(1, 0, -1),  KeyType(-1, 0, -1),
          KeyType(0, 1, 1),  KeyType(0, -1, 1),  KeyType(0, 1, -1),
          KeyType(0, -1, -1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
      nearby_grids_ = {
          KeyType(0, 0, 0),   KeyType(-1, 0, 0),  KeyType(1, 0, 0),
          KeyType(0, 1, 0),   KeyType(0, -1, 0),  KeyType(0, 0, -1),
          KeyType(0, 0, 1),   KeyType(1, 1, 0),   KeyType(-1, 1, 0),
          KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),
          KeyType(-1, 0, 1),  KeyType(1, 0, -1),  KeyType(-1, 0, -1),
          KeyType(0, 1, 1),   KeyType(0, -1, 1),  KeyType(0, 1, -1),
          KeyType(0, -1, -1), KeyType(1, 1, 1),   KeyType(-1, 1, 1),
          KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1),
          KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
    }
  }

  template <int dim, IVoxNodeType node_type, typename PointType>
  void IVox<dim, node_type, PointType>::AddPoints(
      const PointVector& points_to_add) {
    for (size_t i = 0; i < points_to_add.size(); i++) {
      auto key = Pos2Grid(Eigen::Matrix<float, dim, 1>(
          points_to_add[i].x, points_to_add[i].y, points_to_add[i].z));
      auto iter = grids_map_.find(key);
      if (iter == grids_map_.end()) {
        PointType center;
        center.getVector3fMap() = key.template cast<float>()
                                  * options_.resolution_;

        grids_cache_.push_front({key, NodeType(center, options_.resolution_)});
        grids_map_.insert({key, grids_cache_.begin()});

        grids_cache_.front().second.InsertPoint(points_to_add[i]);

        if (grids_map_.size() >= options_.capacity_) {
          grids_map_.erase(grids_cache_.back().first);
          grids_cache_.pop_back();
        }
      } else {
        iter->second->second.InsertPoint(points_to_add[i]);
        grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
        grids_map_[key] = grids_cache_.begin();
      }
    }
  }

  template <int dim, IVoxNodeType node_type, typename PointType>
  Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid(
      const IVox::PtType& pt) const {
    return (pt * options_.inv_resolution_).array().floor().template cast<int>();
  }

}  // namespace faster_lio

#endif
