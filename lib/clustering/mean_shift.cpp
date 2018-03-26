/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///


#include "mean_shift.h"
#include <stdlib.h>
#include <time.h>
#include <algorithm>
namespace MouseTrack {

MeanShift::MeanShift(int k) : _k(k) {
    //empty
}

std::vector<Cluster> MeanShift::operator()(const PointCloud& cloud) const {

	std::vector<Cluster> clustering = std::vector<Cluster>(_k);

  //Initialize RNG
	srand(time(NULL));

	//Assign random cluster to each point in cloud
	for(PointIndex i=0; i<cloud.size(); i++) {
		//Assign index i to random cluster in clustering
		int cluster = rand() % _k;
		clustering[cluster].points().push_back(i);
	}

	//get rid of empty clusters
	int nextInsert = 0;
  for(int i = 0; i < clustering.size(); i++){
    if(!clustering[i].points().empty()){
      clustering[nextInsert] = clustering[i];
      nextInsert++;
    }
  }
	clustering.erase(clustering.begin() + nextInsert, clustering.end());

	return clustering;
}

} //MouseTrack
