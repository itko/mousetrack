/** \file
 * center.h
 *
 *  Created on: Mar 26, 2018
 *      Author: itto
 */

#pragma once

#include "cluster_descriptor.h"
#include "generic/point_cloud.h"

namespace MouseTrack {


class Center : public ClusterDescriptor {
public:
	Center(Coordinate x, Coordinate y, Coordinate z) : _x(0), _y(0), _z(0) {
	}
	~Center(){}
	/// Returns a distance/norm `n` between `this` and `other`.
	/// It should be small for two instances that are "similar" to each other,
	/// and large for two instances that are very different.
	/// Important properties:
	/// 1. 0 <= n <= inf
	/// 2. if this == other -> n == 0
	virtual double compare(const ClusterDescriptor* other) const;
	const Coordinate& x() const;
	const Coordinate& y() const;
	const Coordinate& z() const;

private:
	Coordinate _x;
	Coordinate _y;
	Coordinate _z;
};


} // MouseTrack
