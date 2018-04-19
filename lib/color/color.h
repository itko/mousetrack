/** \file
 * color.h
 *
 *  Created on: Apr 15, 2018
 *      Author: itto
 */

#include "generic/point_cloud.h"
#include <vector>

#pragma once

namespace MouseTrack {

std::vector<std::vector<double>> GenerateNColors(int n);

std::vector<double> GenerateRandomColor();

} // namespace MouseTrack
