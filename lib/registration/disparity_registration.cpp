/// \file
/// Maintainer: Felice Serena
///

#include "disparity_registration.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/log/trivial.hpp>

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
    std::vector<Eigen::Matrix4d> decompositions(frames.size());
    for(size_t i = 0; i < frames.size(); i += 1){
        Eigen::Matrix4d mat = frames[i].rotationCorrection * Ts[i];
        decompositions[i] = mat.inverse();
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
        for(int y = _frame_border - 1; y < disp.rows() - _frame_border; y += 1){
            for(int x = _frame_border - 1; x < disp.cols() - _frame_border; x += 1){
                double disparity = 255*disp(y,x); // disparity is returned between [0,1], but originally stored as [0,255]
                if(disparity < _min_disparity){
                    // just skip those points
                    continue;
                }
                const double invDisparity = 1.0/disparity;
                auto p = cloud[next_insert];
                p.x() = (x + _xshift - f.ccx)*f.baseline*invDisparity;
                p.y() = (y + _yshift - f.ccy)*f.baseline*invDisparity;
                p.z() = f.focallength*f.baseline*invDisparity;

                Eigen::Vector4d tmp = decompositions[i] * Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
                p.x() = tmp[0];
                p.y() = tmp[1];
                p.z() = tmp[2];
                p.intensity() = f.referencePicture(y, x);
                next_insert += 1;
            }
        }
    }
    cloud.resize(next_insert); // shrink to actual number of points
    BOOST_LOG_TRIVIAL(debug) << "expoected points: " << expected_points << ", inserted points: " << next_insert;
    return cloud;
}


double& DisparityRegistration::minDisparity() {
    return _min_disparity;
}

/// Lowest disparity value we accept (we remove points at infinity)
const double& DisparityRegistration::minDisparity() const {
    return _min_disparity;
}

/// Set X shift to correct disparity map position
int& DisparityRegistration::correctingXShift() {
    return _xshift;
}

/// Read X shift to correct disparity map position
const int& DisparityRegistration::correctingXShift() const {
    return _xshift;
}

/// Set Y shift to correct disparity map position
int& DisparityRegistration::correctingYShift() {
    return _yshift;
}

/// Read Y shift to correct disparity map position
const int& DisparityRegistration::correctingYShift() const {
    return _yshift;
}

/// Ignores boder of n pixels around disparity map
int& DisparityRegistration::frameBoder() {
    return _frame_border;
}

/// Ignores boder of n pixels around disparity map
const int& DisparityRegistration::frameBorder() const {
    return _frame_border;
}

} // MouseTrack

