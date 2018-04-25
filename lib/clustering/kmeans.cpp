/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///

#include "kmeans.h"

#include <Eigen/Dense>
#include <boost/log/trivial.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace MouseTrack {

KMeans::KMeans(int k) : _k(k) {
  // empty
}

std::vector<Cluster> KMeans::operator()(const PointCloud &cloud) const {
  BOOST_LOG_TRIVIAL(debug) << "KMeans algorithm started";
  // Convert point cloud to Eigen vectors

  if (cloud.size() == 0) {
    return std::vector<Cluster>();
  }

  // A bit unelegant, it determines the dimension of the points
  size_t dims = cloud[0].eigenVec().size();
  Eigen::MatrixXd points(dims,cloud.size());
  BOOST_LOG_TRIVIAL(debug) << "r" << points.rows() << " c" << points.cols();
  for (PointIndex i = 0; i < cloud.size(); i += 1) {
    points.col(i) = cloud[i].eigenVec();
  }

  // Convert Eigen to OpenCV format. Note, OpenCV KMeans takes points row-wise.
  Eigen::MatrixXd pointsT = points.transpose();
  cv::Mat cvpoints (cloud.size(),dims,CV_32F);
  cv::eigen2cv(pointsT,cvpoints);

  // Output matrices for the OpenCV Kmeans
  cv::Mat cluster_indices, centers;

  // Termination Criteria
  cv::TermCriteria tc ( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, _epsilon, _max_iterations);

  // Apply Kmeans
  cv::kmeans(cvpoints, _k, cluster_indices, tc, _attempts, cv::KMEANS_PP_CENTERS, centers);

  // Make output vector
  std::vector<Cluster> clusters;
  for (int i=0; i < cluster_indices.cols; i++) {
      if (cluster_indices.at<int>(i) + 1 > clusters.size()) {
          clusters.resize(cluster_indices.at<int>(i) + 1);
      }
      clusters[cluster_indices.at<int>(i)].points().push_back(i);
  }



  BOOST_LOG_TRIVIAL(trace) << "KMeans converged! #Clusters: "
                           << clusters.size();
  return clusters;
}

void KMeans::max_iterations(int max_iterations) {_max_iterations = max_iterations;}
const int KMeans::max_iterations() const {return _max_iterations;}

void KMeans::epsilon(double epsilon) {_epsilon = epsilon;}
const double KMeans::epsilon() const {return _epsilon;}

void KMeans::attempts(int attempts) {_attempts = attempts;}
const int KMeans::attempts() const {return _attempts;}

} // namespace MouseTrack
