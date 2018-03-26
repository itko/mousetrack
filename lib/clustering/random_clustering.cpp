/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///


#include "random_clustering.h"
#include <stdlib.h>
namespace MouseTrack {

RandomClustering::RandomClustering(int k) : _k(k) {
    //empty
}

std::vector<Cluster> RandomClustering::operator()(const PointCloud& cloud) const {

	std::vector<Cluster> clustering = std::vector<Cluster>(_k);

	//Assign random cluster to each point in cloud
	for(PointIndex i=0; i<cloud.size(); i++) {
		//Assign index i to random cluster in clustering
		int cluster = rand() % _k;
		clustering[cluster].points().push_back(i);
	}

	//get rid of empty clusters
	for (int i=0; i<clustering.size(); i++) {
		if (clustering[i].points().size() == 0) clustering.erase(clustering.begin() + i);
	}

	return clustering;
}

} //MouseTrack
