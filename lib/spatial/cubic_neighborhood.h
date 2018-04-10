/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "cube_iterator.h"

#include <Eigen/Core>
#include <vector>
#include <unordered_set>
#include <unordered_map>

namespace MouseTrack {

namespace SpatialImpl {

// typedefs
template <int _Dim>
using CellCoordinate = typename Eigen::Matrix<int, _Dim, 1>;

// declarations
template <int _Dim>
class CubicNeighborhoodLayer;

/// Ideally, a neighborhood would correspond to all cells with distance [r,R] to the reference cell
/// (for a [r,R] as tight as possible) (they would build one cube thick shells of a sphere)
/// As a first implementation we'll use the shells of a cube (from inside to outside)
template <int _Dim>
class CubicNeighborhood {
    std::vector<CubicNeighborhoodLayer<_Dim>> _layers;
public:
    /// maxLayer: index of outtest layer you intend to access
    /// 0th layer: only the central cube is generated
    /// 1st layer: coordinates for which the largest component magnitude is equal 1
    CubicNeighborhood(const int maxLayer) {
        CubeIterator<_Dim, CellCoordinate<_Dim>> iter(2*maxLayer+1);
        CubeIterator<_Dim, CellCoordinate<_Dim>> end;
        /// move cube from [0,2*maxLayer] to [-maxLayer, maxLayer]
        CellCoordinate<_Dim> toCenter;
        toCenter.setConstant(_Dim, -maxLayer);
        std::vector<std::vector<CellCoordinate<_Dim>>> layerCells(maxLayer+1);
        for(; iter != end; ++iter){
            auto centered = *iter+toCenter;
            int highestDimension = centered.array().abs().maxCoeff();
            auto i = *iter;
            layerCells[highestDimension].push_back(centered);
        }
        _layers.resize(maxLayer+1);
        for(int i = 0; i < maxLayer+1; i += 1){
            _layers[i] = CubicNeighborhoodLayer<_Dim>(std::move(layerCells[i]));
        }
    }
    int size() const {
        return _layers.size();
    }
    const CubicNeighborhoodLayer<_Dim>& operator[](int l) const {
        return _layers[l];
    }
};

template <int _Dim>
class CubicNeighborhoodLayer {
    int layer;
    double minDist;
    double maxDist;
    /// holds a list of grid cells corresponding to the desired
    /// layer `l`
    std::vector<CellCoordinate<_Dim>> coordinates;
public:
    CubicNeighborhoodLayer(){
        // empty
    }
    /// Construct cells of layer `l`
    CubicNeighborhoodLayer(std::vector<CellCoordinate<_Dim>>&& cells) {
        coordinates = std::move(cells);
        layer = coordinates[0].array().abs().maxCoeff();
        minDist = std::max(0.0, .5 + layer - 1);
        maxDist = std::sqrt(_Dim * std::pow(.5 + layer, 2));
    }
    const CellCoordinate<_Dim> operator[](int i) const {
        return coordinates[i];
    }
    int size() const {
        return coordinates.size();
    }
    /// minimal point distance from center cell measured in cell counts
    double min() const {
        return minDist;
    }
    /// maximal point distance from center cell measured in cell counts
    double max() const {
        return maxDist;
    }
};

} // SpatialImpl

} // MouseTrack
