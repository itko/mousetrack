/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

namespace MouseTrack {

/// This is an abstract class. Derive from this class to create a usable cluster
/// descriptor.
class ClusterDescriptor {
public:
  virtual ~ClusterDescriptor();
  /// Returns a distance/norm `n` between `this` and `other`.
  /// It should be small for two instances that are "similar" to each other,
  /// and large for two instances that are very different.
  /// Important properties:
  /// 1. 0 <= n <= inf
  /// 2. if this == other -> n == 0
  virtual double compare(const ClusterDescriptor *other) const = 0;
};

} // namespace MouseTrack
