/* \file
 * cog.h
 *
 *  Created on: Mar 25, 2018
 *      Author: itto
 */

#pragma once

#include "descripting.h"

namespace MouseTrack {

class CenterOfGravity: public Descripting {
	public:
		/// Takes a cluster and its referencing PointCloud and creates a cluster descriptor
        virtual std::unique_ptr<ClusterDescriptor> operator()(const Cluster& cluster, const PointCloud& cloud) const;
};


} // MouseTrack
