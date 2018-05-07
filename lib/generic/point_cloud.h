/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "types.h"

#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace MouseTrack {

/// A point cloud holds a list of 3D points. Each point can have additional
/// attributes like color intensity.
class PointCloud {
  // Design note: In general, one has to decide betwen two basic concepts:
  // - Structure of Arrays
  // - Array of Structures
  // I would like to go for SoA, since it has the potential to be very efficient
  // (vector operations) But since it is a pain to work with it, I'd like to
  // implement a slim abstraction level (Point) providing accessors and the
  // concept of single points (we basically emulate AoS). Never done this
  // before, should be fun.
public:
  /// type of a label entry:
  ///
  /// 0 means "not this label"
  ///
  /// 1 means "very much this label"
  ///
  /// Values inbetween show insecurity
  typedef double Label;
  class Point;
  class ConstantPoint;
  /// The Point class is a collection of accessors allowing to manipulate the
  /// values inside the PointCloud in an intuitive way.
  class Point {
    // Design note: I tend to keep the hierarchy flat,
    // this way we can implement it by storing a reference to the PointCloud
    // and an index, and nothing more.
    friend class PointCloud;

  private:
    PointCloud &_cloud;
    size_t _index;
    /// Create a Point instance that manipulates the i-th point of cloud
    Point(PointCloud &cloud, size_t i);

  public:
    /// Write access to x coordinate.
    Coordinate &x();

    /// Read access to x coordinate.
    const Coordinate &x() const;

    /// Write access to y coordinate.
    Coordinate &y();

    /// Read access to y coordinate.
    const Coordinate &y() const;

    /// Write access to z coordinate.
    Coordinate &z();

    /// Read access to z coordinate.
    const Coordinate &z() const;

    /// Write access to color r.
    const ColorChannel &r(const ColorChannel &_new);

    /// Read access to color r.
    const ColorChannel &r() const;

    /// Write access to color g.
    const ColorChannel &g(const ColorChannel &_new);

    /// Read access to color g.
    const ColorChannel &g() const;

    /// Write access to color b.
    const ColorChannel &b(const ColorChannel &_new);

    /// Read access to color b.
    const ColorChannel &b() const;

    /// Write access: set `r = g = b = _new`
    ColorChannel intensity(const ColorChannel &_new);

    /// Read access to color intensity.
    ColorChannel intensity() const;

    /// Write access labels
    ///
    /// 0: This point does not hold this label (non existing entries are to
    /// zero)
    ///
    /// 1: This point very much holds this label
    void labels(std::vector<Label> newLabels);

    /// Read access on labels
    const std::vector<Label> &labels() const;

    /// Convert to dx1 Eigen Vector holding all characteristic values
    Eigen::VectorXd characteristic() const;

    /// Position as eigen column vector
    Eigen::Vector3d pos() const;

    /// assignment
    void operator=(const Point &other);

    /// assignment
    void operator=(const ConstantPoint &other);
  };

  /// Exactly the same as `Point` but it only provides read-only access to the
  /// data.
  class ConstantPoint {
    friend class PointCloud;

  private:
    const PointCloud &_cloud;
    size_t _index;
    /// Create a Point instance that manipulates the i-th point of cloud
    ConstantPoint(const PointCloud &cloud, size_t i);

  public:
    /// Read access to x coordinate.
    const Coordinate &x() const;

    /// Read access to y coordinate.
    const Coordinate &y() const;

    /// Read access to z coordinate.
    const Coordinate &z() const;

    /// Read access to color r.
    const ColorChannel &r() const;

    /// Read access to color g.
    const ColorChannel &g() const;

    /// Read access to color b.
    const ColorChannel &b() const;

    /// Read access to color intensity.
    ColorChannel intensity() const;

    /// Read access on labels
    const std::vector<double> &labels() const;

    /// Convert to dx1 Eigen Vector holding all characteristic values
    Eigen::VectorXd characteristic() const;

    /// Position as eigen column vector
    Eigen::Vector3d pos() const;
  };

  PointCloud();

  /// Make space to accomodate n points.
  void resize(size_t n);

  /// Returns number of points stored.
  size_t size() const;

  /// Read-write access to i-th point.
  Point operator[](size_t i);

  /// Read-only access to i-th point.
  const ConstantPoint operator[](size_t i) const;

  /// How many labels are there?
  int labelsDim() const;

  /// How many characteristic dimensions are there?
  int charDim() const;

  /// min corner of bounding box (all characteristic dimensions)
  Eigen::VectorXd charMin() const;

  /// max corner of bounding box (all characteristic dimensions)
  Eigen::VectorXd charMax() const;

  /// min corner of bounding box (only 3d position of points)
  Eigen::Vector3d posMin() const;

  /// max corner of bounding box (only 3d position of points)
  Eigen::Vector3d posMax() const;

private:
  std::vector<Coordinate> _xs;
  std::vector<Coordinate> _ys;
  std::vector<Coordinate> _zs;
  std::vector<ColorChannel> _r;
  std::vector<ColorChannel> _g;
  std::vector<ColorChannel> _b;
  std::vector<std::vector<Label>> _labels;
};

} // namespace MouseTrack
