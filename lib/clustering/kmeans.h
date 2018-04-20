/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "clustering.h"
#include "generic/cluster.h"
#include "spatial/oracle_factory.h"
#include <Eigen/Core>
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

  /// modify factory settings
  OFactory &oracleFactory();

  /// query factory
  const OFactory &oracleFactory() const;

private:
  int _k;
  OFactory _oracleFactory;
};

} // namespace MouseTrack
