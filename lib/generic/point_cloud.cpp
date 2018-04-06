/// \file
/// Maintainer: Felice Serena
///
///


#include "point_cloud.h"

namespace MouseTrack {

// PointCloud implementation

void PointCloud::resize(size_t n) {
    _xs.resize(n);
    _ys.resize(n);
    _zs.resize(n);
    _color.resize(n);
}

size_t PointCloud::size() const {
    return _xs.size();
}

PointCloud::Point PointCloud::operator[](size_t i) {
    return PointCloud::Point(*this, i);
}

const PointCloud::ConstantPoint PointCloud::operator[](size_t i) const {
    return PointCloud::ConstantPoint(*this, i);
}


// Point implementation

PointCloud::Point::Point(PointCloud& cloud, size_t i) : _cloud(cloud), _index(i) {
    // empty
}


Coordinate& PointCloud::Point::x() {
    return _cloud._xs[_index];
}

const Coordinate& PointCloud::Point::x() const {
    return _cloud._xs[_index];
}

Coordinate& PointCloud::Point::y() {
    return _cloud._ys[_index];
}

const Coordinate& PointCloud::Point::y() const {
    return _cloud._ys[_index];
}

Coordinate& PointCloud::Point::z() {
    return _cloud._zs[_index];
}

const Coordinate& PointCloud::Point::z() const {
    return _cloud._zs[_index];
}

ColorChannel& PointCloud::Point::intensity() {
    return _cloud._color[_index];
}

const ColorChannel& PointCloud::Point::intensity() const {
    return _cloud._color[_index];
}

Eigen::VectorXd PointCloud::Point::eigenVec() const {
  return Eigen::Vector4d(x(),y(),z(),intensity());
}


// ConstantPoint implementation

PointCloud::ConstantPoint::ConstantPoint(const PointCloud& cloud, size_t i) : _cloud(cloud), _index(i) {
    // empty
}

const Coordinate& PointCloud::ConstantPoint::x() const {
    return _cloud._xs[_index];
}

const Coordinate& PointCloud::ConstantPoint::y() const {
    return _cloud._ys[_index];
}

const Coordinate& PointCloud::ConstantPoint::z() const {
    return _cloud._zs[_index];
}

const ColorChannel& PointCloud::ConstantPoint::intensity() const {
    return _cloud._color[_index];
}

Eigen::VectorXd PointCloud::ConstantPoint::eigenVec() const {
  return Eigen::Vector4d(x(),y(),z(),intensity());
}



} // MouseTrack
