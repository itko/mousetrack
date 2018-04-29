/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "clustering.h"
#include "generic/cluster.h"
#include "spatial/oracle_factory.h"
#include <Eigen/Core>
#include <mutex>
#include <vector>

namespace MouseTrack {
class KMeans : public Clustering {
public:
  typedef OracleFactory<double> OFactory;
  typedef OFactory::Oracle Oracle;
  typedef Oracle::PointList PointList;
  typedef Oracle::Point Point;

  KMeans(int k);

  virtual std::vector<Cluster> operator()(const PointCloud &cloud) const;

  void K(int k);
  int K() const;

  void convergenceThreshold(double threshold);
  double convergenceThreshold() const;

  /// modify factory settings
  OFactory &oracleFactory();

  /// query factory
  const OFactory &oracleFactory() const;

private:
  int _k;
  double _convergenceThreshold;
  OFactory _oracleFactory;
  mutable std::mutex _oracleMutex;
  mutable OFactory::Point _cachedBoundingBox;
  mutable std::unique_ptr<Oracle> _cachedOracle;
};

} // namespace MouseTrack
