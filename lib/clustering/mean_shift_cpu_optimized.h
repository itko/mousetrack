/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "mean_shift.h"
#include "spatial/uniform_grid.h"
#include <Eigen/Core>
#include <vector>

/// For CPU optimized version of MeanShift
namespace MouseTrack {
class MeanShiftCpuOptimized : public MeanShift {
public:
  /// k is the number of desired clusters
  MeanShiftCpuOptimized(double window_size);

protected:
  virtual std::vector<Eigen::VectorXd>
  convergePoints(const Oracle::PointList &points) const;

  virtual std::vector<Cluster>
  mergePoints(std::vector<Eigen::VectorXd> &points) const;

private:
  typedef UniformGrid4d::PointList PointList;
  /// Performs one iteration of the mean shift algorithm for a single mode
  Eigen::VectorXd iterate_mode(const Eigen::VectorXd mode,
                               const PointList &state) const;
};

} // namespace MouseTrack
