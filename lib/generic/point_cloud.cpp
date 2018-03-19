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


PointCloud::Coord& PointCloud::Point::x() {
    return _cloud._xs[_index];
}

const PointCloud::Coord& PointCloud::Point::x() const {
    return _cloud._xs[_index];
}

PointCloud::Coord& PointCloud::Point::y() {
    return _cloud._ys[_index];
}

const PointCloud::Coord& PointCloud::Point::y() const {
    return _cloud._ys[_index];
}

PointCloud::Coord& PointCloud::Point::z() {
    return _cloud._zs[_index];
}

const PointCloud::Coord& PointCloud::Point::z() const {
    return _cloud._zs[_index];
}

PointCloud::Color& PointCloud::Point::intensity() {
    return _cloud._color[_index];
}

const PointCloud::Color& PointCloud::Point::intensity() const {
    return _cloud._color[_index];
}


// ConstantPoint implementation

PointCloud::ConstantPoint::ConstantPoint(const PointCloud& cloud, size_t i) : _cloud(cloud), _index(i) {
    // empty
}

const PointCloud::Coord& PointCloud::ConstantPoint::x() const {
    return _cloud._xs[_index];
}

const PointCloud::Coord& PointCloud::ConstantPoint::y() const {
    return _cloud._ys[_index];
}

const PointCloud::Coord& PointCloud::ConstantPoint::z() const {
    return _cloud._zs[_index];
}

const PointCloud::Color& PointCloud::ConstantPoint::intensity() const {
    return _cloud._color[_index];
}


} // MouseTrack
