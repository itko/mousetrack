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

void PointCloud::resize(size_t n, size_t labelsCount) {
  _xs.resize(n);
  _ys.resize(n);
  _zs.resize(n);
  _r.resize(n);
  _g.resize(n);
  _b.resize(n);
  _labels.resize(labelsCount, n);
}

size_t PointCloud::size() const { return _xs.size(); }

PointCloud::Point PointCloud::operator[](size_t i) {
  return PointCloud::Point(*this, i);
}

const PointCloud::ConstantPoint PointCloud::operator[](size_t i) const {
  return PointCloud::ConstantPoint(*this, i);
}

int PointCloud::labelsDim() const { return _labels.rows(); }

int PointCloud::charDim() const {
  // position: 3
  // intensity: 1
  // labels: labelsDim()
  return 3 + 1 + labelsDim();
}

Eigen::Vector3d PointCloud::posMin() const {
  Eigen::Vector3d min;
  min.setConstant(3, std::numeric_limits<double>::max());

  for (PointIndex i = 0; i < size(); i += 1) {
    const auto &p = (*this)[i];
    min = min.array().min(p.pos().array());
  }
  return min;
}

Eigen::Vector3d PointCloud::posMax() const {
  Eigen::Vector3d max;
  max.setConstant(3, std::numeric_limits<double>::min());

  for (PointIndex i = 0; i < size(); i += 1) {
    const auto &p = (*this)[i];
    max = max.array().max(p.pos().array());
  }
  return max;
}

Eigen::VectorXd PointCloud::charMin() const {
  Eigen::VectorXd min;
  min.setConstant(charDim(), std::numeric_limits<double>::max());

  for (PointIndex i = 0; i < size(); i += 1) {
    const auto &p = (*this)[i];
    min = min.array().min(p.characteristic().array());
  }
  return min;
}

Eigen::VectorXd PointCloud::charMax() const {
  Eigen::VectorXd max;
  max.setConstant(charDim(), std::numeric_limits<double>::min());

  for (PointIndex i = 0; i < size(); i += 1) {
    const auto &p = (*this)[i];
    max = max.array().max(p.pos().array());
  }
  return max;
}

// Point implementation

PointCloud::Point::Point(PointCloud &cloud, size_t i)
    : _cloud(cloud), _index(i) {
  // empty
}

Coordinate &PointCloud::Point::x() { return _cloud._xs[_index]; }

const Coordinate &PointCloud::Point::x() const { return _cloud._xs[_index]; }

Coordinate &PointCloud::Point::y() { return _cloud._ys[_index]; }

const Coordinate &PointCloud::Point::y() const { return _cloud._ys[_index]; }

Coordinate &PointCloud::Point::z() { return _cloud._zs[_index]; }

const Coordinate &PointCloud::Point::z() const { return _cloud._zs[_index]; }

const ColorChannel &PointCloud::Point::r(const ColorChannel &_new) {
  _cloud._r[_index] = _new;
  return _cloud._r[_index];
}

const ColorChannel &PointCloud::Point::r() const { return _cloud._r[_index]; }

const ColorChannel &PointCloud::Point::g(const ColorChannel &_new) {
  _cloud._g[_index] = _new;
  return _cloud._g[_index];
}

const ColorChannel &PointCloud::Point::g() const { return _cloud._g[_index]; }

const ColorChannel &PointCloud::Point::b(const ColorChannel &_new) {
  _cloud._b[_index] = _new;
  return _cloud._b[_index];
}

const ColorChannel &PointCloud::Point::b() const { return _cloud._b[_index]; }

ColorChannel PointCloud::Point::intensity() const {
  return (_cloud._r[_index] + _cloud._g[_index] + _cloud._b[_index]) / 3.0;
}

ColorChannel PointCloud::Point::intensity(const ColorChannel &_new) {
  _cloud._r[_index] = _new;
  _cloud._g[_index] = _new;
  _cloud._b[_index] = _new;
  return _new;
}

void PointCloud::Point::labels(const PointCloud::LabelVec &newLabels) {
  _cloud._labels.col(_index) = newLabels;
}

void PointCloud::Point::labels(PointCloud::LabelVec &&newLabels) {
  _cloud._labels.col(_index) = std::move(newLabels);
}

PointCloud::LabelVecConstOut PointCloud::Point::labels() const {
  // easiest way to add `const`?
  const auto &l = _cloud._labels;
  return l.col(_index);
}

Eigen::VectorXd PointCloud::Point::characteristic() const {
  Eigen::VectorXd result(_cloud.charDim());
  result[0] = x();
  result[1] = y();
  result[2] = z();
  result[3] = intensity();
  result.block(4, 0, _cloud.labelsDim(), 1) = labels();
  return result;
}

Eigen::Vector3d PointCloud::Point::pos() const {
  return Eigen::Vector3d(x(), y(), z());
}

void PointCloud::Point::operator=(const PointCloud::Point &o) {
  x() = o.x();
  y() = o.y();
  z() = o.z();
  r(o.r());
  g(o.g());
  b(o.b());
  labels(o.labels());
}

void PointCloud::Point::operator=(const PointCloud::ConstantPoint &o) {
  x() = o.x();
  y() = o.y();
  z() = o.z();
  r(o.r());
  g(o.g());
  b(o.b());
  labels(o.labels());
}

// ConstantPoint implementation

PointCloud::ConstantPoint::ConstantPoint(const PointCloud &cloud, size_t i)
    : _cloud(cloud), _index(i) {
  // empty
}

const Coordinate &PointCloud::ConstantPoint::x() const {
  return _cloud._xs[_index];
}

const Coordinate &PointCloud::ConstantPoint::y() const {
  return _cloud._ys[_index];
}

const Coordinate &PointCloud::ConstantPoint::z() const {
  return _cloud._zs[_index];
}

const ColorChannel &PointCloud::ConstantPoint::r() const {
  return _cloud._r[_index];
}

const ColorChannel &PointCloud::ConstantPoint::g() const {
  return _cloud._g[_index];
}

const ColorChannel &PointCloud::ConstantPoint::b() const {
  return _cloud._b[_index];
}

ColorChannel PointCloud::ConstantPoint::intensity() const {
  return (_cloud._r[_index] + _cloud._g[_index] + _cloud._b[_index]) / 3.0;
}

PointCloud::LabelVecConstOut PointCloud::ConstantPoint::labels() const {
  return _cloud._labels.col(_index);
}

Eigen::VectorXd PointCloud::ConstantPoint::characteristic() const {
  Eigen::VectorXd result(_cloud.charDim());
  result[0] = x();
  result[1] = y();
  result[2] = z();
  result[3] = intensity();
  result.block(4, 0, _cloud.labelsDim(), 1) = labels();
  return result;
}

Eigen::Vector3d PointCloud::ConstantPoint::pos() const {
  return Eigen::Vector3d(x(), y(), z());
}

} // namespace MouseTrack
