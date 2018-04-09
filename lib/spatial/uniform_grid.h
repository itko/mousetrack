

/// A neighborhood tries to describe a set of grid cells that have roughly the same distance
/// to a central reference cell
/// Ideally, a neighborhood would correspond to all cells with distance [r,R] to the reference cell
/// (for a [r,R] as tight as possible) (they would bild one cube thick shells of a sphere)
/// Since this seems quite involved mathematically, we go with a rougher implementation, where we use
/// hollow-cube shaped layers of thickness one cube
class Neighborhood {
    const int layer;
    const double minDist;
    const double maxDist;
    std::vector<Eigen::Vector3i> coordinates;
public:
    Neighborhood(int l) : layer(l),
    minDist(.5 + (l-1)), maxDist(sqrt(.5*.5*.5 + 3*l*l)) {
        std::vector<Eigen::Vector3i>& n = coordinates;
        if(l == 0){
            n.push_back(Eigen::Vector3i(0,0,0));
            return;
        }
        // x-planes of cube
        for(int y = -l; y <= l; y += 1){
            for(int z = -l; z <= l; z += 1){
                n.push_back(Eigen::Vector3i(l, y, z));
                n.push_back(Eigen::Vector3i(-l, y, z));
            }
        }
        // y-planes
        for(int x = -l+1; x < l; x += 1){
            for(int z = -l; z <= l; z += 1){
                n.push_back(Eigen::Vector3i(x,l,z));
                n.push_back(Eigen::Vector3i(x,-l,z));
            }
        }
        // z-planes
        for(int x = -l+1; x < l; x += 1){
            for(int y = -l+1; y < l; y += 1){
                n.push_back(Eigen::Vector3i(x,y,l));
                n.push_back(Eigen::Vector3i(x,y,-l));
            }
        }
        int w = 2*l+1; // cube width
        assert(n.size() == w*w*w - (w-2)*(w-2)*(w-2));
    }
    const std::vector<Eigen::Vector3i>& coords() const {
        return coordinates;
    }
    // minimal point distance from center cell measured in cell counts
    double min() const {
        return minDist;
    }
    // maximal point distance from center cell measured in cell counts
    double max() const {
        return maxDist;
    }
};


class UniformGrid : public SpatialOracle {
private:
    const Eigen::MatrixXd& Ps;
    Eigen::RowVector3d bb_min;
    Eigen::RowVector3d bb_max;
    Eigen::RowVector3d bb_size;
    const double cellWidth;
    mutable std::vector<Neighborhood> neighborhoods;
    Eigen::Vector3i dimCellCount;
    // Improvements: replace three nested vectors with a 3D Eigen-Tensor
    std::vector<std::vector<std::vector<std::vector<size_t>>>> gridIndex;
    Eigen::Vector3i indexOfPosition(const Eigen::RowVector3d& p) const {
        Eigen::Vector3i index;
        for(size_t i = 0; i < 3; i += 1){
            double diff = (p[i] - bb_min[i])/cellWidth;
            double upper = dimCellCount[i] - 1.0;
            index[i] = std::floor(std::min(std::max(0.0, diff), upper));
        }
        return index;
    }

    const Neighborhood& neighborhood(size_t layer) const {
        if(layer < neighborhoods.size()){
            return neighborhoods[layer];
        }
        for(size_t i = neighborhoods.size(); i <= layer; i += 1){
            // create neighborhood of layer i
            neighborhoods.push_back(Neighborhood(i));
        }
        return neighborhoods[layer];
    }
    size_t maxLayer() const {
        return dimCellCount.maxCoeff();
    }
    bool in_bb(const Eigen::Vector3i cell) const {
        for(size_t i = 0; i < 3; i += 1){
            if(cell[i] < 0 || dimCellCount[i] <= cell[i]){
                return false;
            }
        }
        return true;
        //auto tmp = dimCellCount - cell;
        //return 0 <= cell.minCoeff() && tmp.minCoeff() >= 1;
    }
public:
    UniformGrid(const Eigen::MatrixXd& srcData) :
            Ps(srcData),
            bb_min(srcData.colwise().minCoeff()),
            bb_max(srcData.colwise().maxCoeff()),
            bb_size(bb_max - bb_min),
            cellWidth(bb_size.minCoeff()/40.0) // Improvement option: get better heuristic
    {
        bb_min -= bb_size * .1;
        bb_max += bb_size * .1;
        for(size_t i = 0; i < 3; i += 1){
            dimCellCount[i] = std::ceil(bb_size[i]/cellWidth);
        }

        // create grid
        gridIndex.resize(dimCellCount[0]);
        for(auto& x : gridIndex){
            x.resize(dimCellCount[1]);
            for(auto& y : x){
                y.resize(dimCellCount[2]);
            }
        }
        // fill grid with indices
        for(size_t i = 0; i < Ps.rows(); i += 1){
            auto j = indexOfPosition(Ps.row(i));
            gridIndex[j[0]][j[1]][j[2]].push_back(i);
        }
    }
    /// finds the nearest neighbor for a given point p
    virtual size_t find_closest(const Eigen::RowVector3d& p) const {
        // idea: we define hollow cubes around the cell containing p, called layers
        // we search the layers from the inside to the outside
        // if we have a closest candidate, we stop as soon as our active layer's closest point is above our candidate
        auto zeroCell = indexOfPosition(p);
        size_t minIndex = -1;
        double minDist = std::numeric_limits<double>::infinity(); // squared norm between p and closest candidate
        size_t maxL = maxLayer(); // there's no point to look outside of the BB
        for(size_t l = 0; l < maxL; l += 1){
            const Neighborhood& n = neighborhood(l);
            double nDist = (n.min() - 1)*cellWidth;
            double cDist = std::sqrt(minDist);
            if(cDist < nDist){
                // the closest point in the layer is furhter away than our candidate
                // (we need to subtract 1 to account for points p sitting in the corner of the zeroCell
                // we can stop
                break;
            }
            for(size_t i = 0; i < n.coords().size(); i += 1){
                auto cell = zeroCell + n.coords()[i];
                // skip, if outside of bounding box
                if(!in_bb(cell)){
                    continue;
                }
                // we are inside the bounding box and have to check against all verts in the cell
                const auto& candidates = gridIndex[cell[0]][cell[1]][cell[2]];
                for(size_t c : candidates){
                    double dist = (Ps.row(c) - p).squaredNorm();
                    if(dist < minDist){
                        minDist = dist;
                        minIndex = c;
                    }
                }
            }
        }
        return minIndex;
    }
    virtual std::vector<size_t> find_in_range(const Eigen::RowVector3d& p, const double h) const {
        std::vector<size_t> range;
        auto zeroCell = indexOfPosition(p);
        size_t maxL = maxLayer();
        const double h2 = h*h;
        for(size_t l = 0; l < maxL; l += 1){
            const Neighborhood& n = neighborhood(l);
            if(h < (n.min()-1)*cellWidth){
                break;
            }
            for(size_t i = 0; i < n.coords().size(); i += 1){
                auto cell = zeroCell + n.coords()[i];
                // skip, if outside of bounding box
                if(!in_bb(cell)){
                    continue;
                }
                // we are inside the bounding box and have to check against all verts in the cell
                const auto& candidates = gridIndex[cell[0]][cell[1]][cell[2]];
                for(size_t c : candidates){
                    double dist = (Ps.row(c) - p).squaredNorm();
                    if(dist <= h2){
                        range.push_back(c);
                    }
                }

            }
        }
        return range;
    }
};
