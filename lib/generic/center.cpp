/** \file
 * center.cpp
 *
 *  Created on: Mar 26, 2018
 *      Author: itto
 */

#include "center.h"
#include <cmath>

namespace MouseTrack {

const Coordinate& Center::x() const {
	return this->_x;
}

const Coordinate& Center::y() const {
	return this->_y;
}

const Coordinate& Center::z() const {
	return this->_z;
}

double Center::compare(const ClusterDescriptor* other) const {
	const Center * otherCenter = dynamic_cast<const Center*>(other);
	// Get Euclidean distance
	double err = pow((this->_x - otherCenter->x()),2) + pow((this->_y - otherCenter->y()),2) + pow((this->_z - otherCenter->z()),2);
	double n = sqrt(err);
	return n;
};


} // MouseTrack


