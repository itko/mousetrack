/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include <cstddef>
#include <Eigen/Core>

namespace MouseTrack {

/// Used to index elements of a point cloud, don't expect it to support negative numbers
typedef size_t PointIndex;

/// Used to reference frame numbers, negative numbers are supported
/// (example: -1 might mean we don't know the frame number)
typedef int FrameNumber;

/// Used to reference the stream number, each disparity map has a unique stream number
typedef int StreamNumber;

/// If parameters or so, need to be stored, which precision should be used?
typedef double Precision;

/// Defines precision of a single coordinate component.
typedef double Coordinate;

/// Defines precision of a single color component
typedef float ColorChannel;

/// Represents an input picture from a camera stream
typedef Eigen::MatrixXd Picture;

} // MouseTrack
