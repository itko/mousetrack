/// \file
/// Maintainer: Felice Serena
///

#pragma once

namespace MouseTrack {

namespace SpatialImpl {

/// interface template for neighborhood data structures
template <typename Layer>
class Neighborhood {
public:
    /// number of available layers
    virtual int size() const;
    /// access to layer `l` for querying
    virtual const Layer& operator[](int l) const;
};

/// A neighborhood layer tries to describe a set of grid cells that have roughly the same distance
/// to a central reference cell
template <typename CellCoordinate>
class NeighborhoodLayer {
public:
    /// give i-th neighbor cell in layer
    virtual const CellCoordinate operator[](int i) const = 0;

    /// give number of available neighbor cells
    virtual int size() const = 0;

    /// minimal point distance from center cell measured in cell counts
    virtual double min() const = 0;

    /// maximal point distance from center cell measured in cell counts
    virtual double max() const = 0;
};

} // SpatialImpl
} // MouseTrack
