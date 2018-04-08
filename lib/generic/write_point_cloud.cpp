/// \file
/// Maintainer: Felice Serena
///
///

#include "write_point_cloud.h"

#include <fstream>

namespace MouseTrack {

void write_point_cloud(const std::string& path, const PointCloud& cloud) {
    std::ofstream out;
    out.open(path.c_str());
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << cloud.size() << "\n";
    out << "property double x\n";
    out << "property double y\n";
    out << "property double z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "end_header\n";
    for(size_t i = 0; i < cloud.size(); i += 1){
        const auto p = cloud[i];
        int in = 255*p.intensity();
        out << p.x() << " " << p.y() << " " << p.z() << " " << in << " " << in << " " << in <<  "\n";
    }
    out.close();
}

} // MouseTrack
