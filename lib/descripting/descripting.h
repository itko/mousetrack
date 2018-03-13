/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include<vector>

namespace Mousetrack {


class Descripting {
	public:
		virtual ClusterDescriptor operator(const std::vector<Cluster>& clusters) const = 0;
};


}
