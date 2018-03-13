/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include<vector>

namespace Mousetrack {


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
		Coord& x();
		const Coord& x() const;
		Coord& y();
		const Coord& y() const;
		Coord& z();
		const Coord& z() const;
		Color& r();
		const Color& r() const;
		Color& g();
		const Color& g() const;
		Color& b();
		const Color& b() const;
	};
	void resize(size_t n);
	size_t size() const;
	Point& operator[size_t i];
	const Point& operator[size_t i] const;
};


}
