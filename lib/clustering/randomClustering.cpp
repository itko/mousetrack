/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///


#include "randomClustering.h"
#include <stdlib.h>

namespace MouseTrack {

RandomClustering::RandomClustering(int k) : _k(k) {
    //empty
}

std::vector<Cluster> RandomClustering::operator()(const PointCloud& cloud) {

	std::vector<Cluster> clustering = std::vector<Cluster>(_k);
	

	for(PointIndex i=0; i<cloud.size(); i++) {
		//Assign index i to random cluster in clustering
		int cluster = rand() % _k;
		clustering[i].points().push_back(i);
	}

	return clustering;
}

} //MouseTrack
