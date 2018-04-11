/// \file
/// Maintainer: Felice Serena
///
///


#include "point_cloud.h"

namespace MouseTrack {

// PointCloud implementation

PointCloud::PointCloud() {
    // empty
}

void PointCloud::resize(size_t n) {
    _xs.resize(n);
    _ys.resize(n);
    _zs.resize(n);
    _r.resize(n);
    _g.resize(n);
    _b.resize(n);
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

const ColorChannel& PointCloud::Point::r(const ColorChannel& _new) {
    _cloud._r[_index] = _new;
    return _cloud._r[_index];
}

const ColorChannel& PointCloud::Point::r() const {
    return _cloud._r[_index];
}

const ColorChannel& PointCloud::Point::g(const ColorChannel& _new) {
    _cloud._g[_index] = _new;
    return _cloud._g[_index];
}

const ColorChannel& PointCloud::Point::g() const {
    return _cloud._g[_index];
}

const ColorChannel& PointCloud::Point::b(const ColorChannel& _new) {
    _cloud._b[_index] = _new;
    return _cloud._b[_index];
}

const ColorChannel& PointCloud::Point::b() const {
    return _cloud._b[_index];
}

ColorChannel PointCloud::Point::intensity() const {
    return (_cloud._r[_index] + _cloud._g[_index] + _cloud._b[_index])/3.0;
}

ColorChannel PointCloud::Point::intensity(const ColorChannel& _new) {
    _cloud._r[_index] = _new;
    _cloud._g[_index] = _new;
    _cloud._b[_index] = _new;
    return _new;
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

const ColorChannel& PointCloud::ConstantPoint::r() const {
    return _cloud._r[_index];
}

const ColorChannel& PointCloud::ConstantPoint::g() const {
    return _cloud._g[_index];
}

const ColorChannel& PointCloud::ConstantPoint::b() const {
    return _cloud._b[_index];
}

ColorChannel PointCloud::ConstantPoint::intensity() const {
    return (_cloud._r[_index] + _cloud._g[_index] + _cloud._b[_index])/3.0;
}

Eigen::VectorXd PointCloud::ConstantPoint::eigenVec() const {
  return Eigen::Vector4d(x(),y(),z(),intensity());
}



} // MouseTrack
