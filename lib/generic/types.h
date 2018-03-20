/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include <cstddef>

namespace MouseTrack {

/// Used to index elements of a point cloud, don't expect it to support negative numbers
typedef size_t PointIndex;

/// Used to reference frame numbers, negative numbers are supported
/// (example: -1 might mean we don't know the frame number)
typedef int FrameNumber;

/// Defines precision of a single coordinate component.
typedef double Coordinate;

/// Defines precision of a single color component
typedef float ColorChannel;

} // MouseTrack
