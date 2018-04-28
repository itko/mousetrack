/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///

#include "kmeans.h"
#include "spatial/uniform_grid.h"
#include <Eigen/Dense>
#include <boost/log/trivial.hpp>
#include <iostream>

namespace MouseTrack {

KMeans::KMeans(int K) : _k(K) {
  // empty
}

std::vector<Cluster> KMeans::operator()(const PointCloud &cloud) const {
  BOOST_LOG_TRIVIAL(trace) << "KMeans algorithm started";
  // Convert point cloud to Eigen vectors

  if (cloud.size() == 0) {
    return std::vector<Cluster>();
  }

  const int dims = cloud[0].eigenVec().size();
  PointList means(dims, K());
  PointList prevMeans;

  PointList points;
  points.resize(dims, cloud.size());
  for (PointIndex i = 0; i < cloud.size(); i += 1) {
    auto v = cloud[i].eigenVec();
    points.col(i) = v;
  }

  // TODO: get kmeans++ initialization
  means = points.block(0, 0, dims, K());

  Eigen::VectorXd min = points.rowwise().minCoeff();
  Eigen::VectorXd max = points.rowwise().maxCoeff();
  Eigen::VectorXd bb_size = max - min;
  Precision change = std::numeric_limits<Precision>::max();

  BOOST_LOG_TRIVIAL(debug) << "KMeans: bb: " << bb_size;

  std::unique_ptr<Oracle> oraclePtr;
  {
    OFactory::Query q;
    q.dimensions = bb_size.size();
    q.bb_size = &bb_size;
    oraclePtr = oracleFactory().forQuery(q);
  }
  Oracle &oracle = *oraclePtr;

  std::vector<Cluster> clusters;

  BOOST_LOG_TRIVIAL(debug) << "KMeans: Starting iterations.";
  do {
    // assign clusters
    BOOST_LOG_TRIVIAL(trace) << "KMeans: Assigning clusters";
    oracle.compute(means);
    clusters = std::vector<Cluster>(K());

    for (int i = 0; i < points.cols(); ++i) {
      std::vector<size_t> cs = oracle.find_closest(points.col(i), 1);
      if (cs.empty()) {
        oracle.find_closest(points.col(i), 1);
        BOOST_LOG_TRIVIAL(error)
            << "Couldn't find nearest mean for point " << i << " ("
            << points.col(i) << "), assigning to 1";
        cs.push_back(1);
      }
      clusters[cs[0]].points().push_back(i);
    }

    // calculate new cluster centers
    BOOST_LOG_TRIVIAL(trace) << "KMeans: Assigning centroids";
    prevMeans = means;
    for (PointIndex c = 0; c < clusters.size(); ++c) {
      PointList assigned(dims, clusters[c].points().size());
      for (PointIndex i = 0; i < clusters[c].points().size(); ++i) {
        int pi = clusters[c].points()[i];
        assigned.col(i) = points.col(pi);
      }
      Eigen::VectorXd mean = assigned.array().rowwise().sum() / assigned.cols();
      means.col(c) = mean;
    }
    change = (prevMeans - means).norm();
    BOOST_LOG_TRIVIAL(trace) << "change: " << change;
  } while (change > 0.03);

  return clusters;
}

void KMeans::K(int k) { _k = k; }
int KMeans::K() const { return _k; }

KMeans::OFactory &KMeans::oracleFactory() { return _oracleFactory; }

const KMeans::OFactory &KMeans::oracleFactory() const { return _oracleFactory; }

} // namespace MouseTrack
