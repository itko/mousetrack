/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///


#include "mean_shift.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
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

double MeanShift::euclidean_distance_squared(const Point& x, const Point& y) {
	return 	(x.x() - y.x())*(x.x() - y.x()) +
					(x.y() - y.y())*(x.y() - y.y()) +
					(x.z() - y.z())*(x.z() - y.z()) +
					(x.intensity() - y.intensity())*(x.intensity() - y.intensity());
}

double MeanShift::apply_gaussian_kernel(const PointCloud& cloud, const PointIndex& point, const GaussianKernel& kernel) {
	// Euclidean distance squared btw. mean of kernel and point of interest
	const double d = euclidean_distance_squared(kernel.mean, cloud[point]);
	return exp(-d/(2*kernel.variance));
}

} //MouseTrack
