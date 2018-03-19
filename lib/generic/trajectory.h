/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include <map>
#include <memory>
#include <Eigen/Core>

namespace MouseTrack {


/// Describes a path in 3D space
class Trajectory {
public:
    /// Read-Write access to control points.
    /// Key: frame number
    /// Value: point position in 3D space, might be null if no correspondence could be found in a certain frame
    std::map<size_t, std::shared_ptr<Eigen::Vector3d>> controlPoints();
    /// Read access to control points.
    const std::map<size_t, std::shared_ptr<Eigen::Vector3d>> controlPoints() const;
private:
    std::map<size_t, std::shared_ptr<Eigen::Vector3d>> _controlPoints;
};


} // MouseTrack
