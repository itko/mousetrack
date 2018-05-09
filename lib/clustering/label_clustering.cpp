/// \file
/// Maintainer: Felice Serena
///
///

#include "label_clustering.h"

namespace MouseTrack {

std::vector<Cluster> LabelClustering::
operator()(const PointCloud &cloud) const {
  int clusterCount = cloud.labelsDim() + 1;
  std::vector<Cluster> clusters(clusterCount);
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto p = cloud[i];
    const auto &labels = p.labels();
    auto max = std::max_element(labels.begin(), labels.end());
    if (*max > _rejectionThreshold) {
      clusters[max - labels.begin()].points().push_back(i);
    } else {
      // rubbish is collected in the last cluster
      clusters.back().points().push_back(i);
    }
  }
  return clusters;
}

} // namespace MouseTrack
