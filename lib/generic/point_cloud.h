/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include <cstddef>
#include <vector>

namespace MouseTrack {


/// A point clouds holds a list of 3D points. Each point can have additional attributes like color intensity.
class PointCloud {
	// Design note: In general, one has to decide betwen two basic concepts:
	// - Structure of Arrays
	// - Array of Structures
	// I would like to go for SoA, since it has the potential to be very efficient (vector operations)
	// But since it is a pain to work with it, I'd like to implement a slim abstraction level (Point)
	// providing accessors and the concept of single points (we basically emulate AoS).
	// Never done this before, should be fun.
public:
	typedef double Coord;
	typedef float Color;
	/// The Point class is a collection of accessors allowing to manipulate the values inside the PointCloud in an intuitive way.
	class Point {
		// Design note: I tend to keep the hierarchy flat, 
		// this way we can implement it by storing a reference to the PointCloud
		// and an index, and nothing more.
        friend class PointCloud;
    private:
        PointCloud& _cloud;
        size_t _index;
        /// Create a Point instance that manipulates the i-th point of cloud
        Point(PointCloud& cloud, size_t i);
    public:
        /// Write access to x coordinate.
		Coord& x();
        /// Read access to x coordinate.
		const Coord& x() const;
        /// Write access to y coordinate.
		Coord& y();
        /// Read access to y coordinate.
		const Coord& y() const;
        /// Write access to z coordinate.
		Coord& z();
        /// Read access to z coordinate.
		const Coord& z() const;
        /// Write access to color intensity.
        Color& intensity();
        /// Read access to color intensity.
        const Color& intensity() const;
    };
    /// Exactly the same as `Point` but it only provides read-only access to the data.
    class ConstantPoint {
        // Design note: I tend to keep the hierarchy flat,
        // this way we can implement it by storing a reference to the PointCloud
        // and an index, and nothing more.
        friend class PointCloud;
    private:
        const PointCloud& _cloud;
        size_t _index;
        /// Create a Point instance that manipulates the i-th point of cloud
        ConstantPoint(const PointCloud& cloud, size_t i);
    public:
        /// Read access to x coordinate.
        const Coord& x() const;
        /// Read access to y coordinate.
        const Coord& y() const;
        /// Read access to z coordinate.
        const Coord& z() const;
        /// Read access to color intensity.
        const Color& intensity() const;
    };
    /// Make space to accomodate n points.
	void resize(size_t n);
    /// Returns number of points stored.
    size_t size() const;
    /// Read-write access to i-th point.
    Point operator[](size_t i);
    /// Read-only access to i-th point.
    const ConstantPoint operator[](size_t i) const;
private:
    std::vector<Coord> _xs;
    std::vector<Coord> _ys;
    std::vector<Coord> _zs;
    std::vector<Color> _color;
};


} // MouseTrack
