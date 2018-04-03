/// \file
/// Maintainer: Felice Serena
///

#include "disparity_registration.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace MouseTrack {

PointCloud DisparityRegistration::operator()(const FrameWindow& window) const {
    const auto& frames = window.frames();

    // how many points will there be at most?
    int expected_points = 0;
    for(const auto& f : frames){
        expected_points += f.normalizedDisparityMap.zMap().size();
    }

    // absolute transformation matrix relative to first camera
    std::vector<Eigen::Matrix4d> Ts(frames.size());
    Ts[0] = Eigen::Matrix4d::Identity();
    for(size_t i = 1; i < frames.size(); i += 1){
        Ts[i] = frames[i].camChainPicture*frames[i-1].camChainDisparity*Ts[i-1];
    }
    std::vector<Eigen::ColPivHouseholderQR<Eigen::Matrix4d>> decompositions(frames.size());
    for(size_t i = 0; i < frames.size(); i += 1){
        Eigen::Matrix4d mat = frames[i].rotationCorrection * Ts[i];
        decompositions[i] = mat.colPivHouseholderQr();
    }

    // Allocate point cloud enough large to capture all points
    PointCloud cloud;
    cloud.resize(expected_points);
    int next_insert = 0;
    // go through each frame, converting the disparity values to 3d points relative to first camera
    for(size_t i = 0; i < frames.size(); i += 1){
        const auto& f = frames[i];
        const auto& disp = f.normalizedDisparityMap.zMap();
        // convert each pixel
        for(int y = frame_border - 1; y < disp.rows() - frame_border; y += 1){
            for(int x = frame_border - 1; x < disp.cols() - frame_border; x += 1){
                double disparity = disp(y,x);
                if(disparity < min_disparity){
                    // just skip those points
                    continue;
                }
                const double invDisparity = 1.0/disparity;
                auto p = cloud[next_insert];
                p.x() = (x + xshift - f.ccx)*f.baseline*invDisparity;
                p.y() = (y + yshift - f.ccy)*f.baseline*invDisparity;
                p.z() = f.focallength*f.baseline*invDisparity;
                // this is probably nothing else than the inverse of r*T applied to p
                // one might be able to optimize this by calculating a vecor of inverses
                // but ColPivHousholderQR does probably a similar thing under the hood
                // TODO: for optimization phase: there might be more efficent solutions to this
                Eigen::Vector4d tmp = decompositions[i].solve(Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0));
                p.x() = tmp[0];
                p.y() = tmp[1];
                p.z() = tmp[2];
                p.intensity() = f.referencePicture(y, x);
                next_insert += 1;
            }
        }
    }
    cloud.resize(next_insert); // shrink to actual number of points
    return cloud;
}

} // MouseTrack

