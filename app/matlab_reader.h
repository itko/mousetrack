/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "generic/types.h"
#include "generic/disparity_map.h"
#include "generic/frame.h"
#include <set>
#include <map>
#include <Eigen/Core>
#include <boost/filesystem.hpp>

namespace MouseTrack {

namespace fs = boost::filesystem;


/// The MatlabReader is a very specialized class, that reads preprocessed disparity maps and
/// Parameter files as created by the `extract_camera_streams.m` script.
/// The expected file format and folder structures are hard coded.
/// Takes the path to a folder corresponding to a bag file (via constructor) and gives
/// access to the data via its methods.
///
/// Convention for methods: always first stream, then frame number.
class MatlabReader {
public:
    MatlabReader(fs::path root_directory);

    /// true if constructor decided, the file system is formatted properly, false otherwise
    bool valid() const;

    /// Index of first existing stream
    StreamNumber beginStream() const;

    /// C++ convention: points to the first non existing stream at the end
    StreamNumber endStream() const;

    /// index of first existing frame
    FrameNumber beginFrame() const;

    /// Index of first non existing frame at the end
    FrameNumber endFrame() const;

    /// A list of all paths (files, directories, ...) that were skipped during the preflight
    const std::vector<fs::path>& ignoredPaths() const;

    /// A list of all channels that are ignored (files that match the regular expression
    /// for a sequence of stream images, but has an unknown base name).
    const std::vector<std::string>& ignoredChannels() const;

    /// Fetch normalized depth map for frame f and stream s
    DisparityMap normalizedDisparityMap(StreamNumber s, FrameNumber f) const;

    /// Fetch depth map as stored in bag file for frame f and stream s
    DisparityMap rawDisparityMap(StreamNumber s, FrameNumber f) const;

    /// Fetch channel parameters (focallength, baseline, ccx, ccy)
    Eigen::MatrixXd channelParameters(FrameNumber f) const;

    /// Fetch left camera picture for frame f and stream s
    Picture picture(StreamNumber s, FrameNumber f) const;

    /// Fetch rotation corrections for cameras
    std::vector<Eigen::Matrix4d> rotationCorrections() const;

    /// Fetch the camera chain (8 rotation matrices that define how the cameras are positioned to each other)
    std::vector<Eigen::Matrix4d> camchain() const;

    /// Fetch all data belonging to frame f of stream s
    Frame frame(StreamNumber s, FrameNumber f) const;
private:
    struct StaticParameters {
        std::set<fs::path> rotationCorrectionsPath;
        std::vector<Eigen::Matrix4d> rotationCorrections;
        std::set<fs::path> camchainPath;
        std::vector<Eigen::Matrix4d> camchain;
    };
    struct ElementKey {
        ElementKey(StreamNumber s, FrameNumber f, std::string c) : stream(s), frame(f), channel(c) {
            // empty
        }
        StreamNumber stream;
        FrameNumber frame;
        std::string channel;
        bool operator<(const ElementKey& other) const;
        bool operator==(const ElementKey& other) const;
    };

    fs::path _root;
    bool _valid;
    StreamNumber _beginStream;
    StreamNumber _endStream;
    FrameNumber _beginFrame;
    FrameNumber _endFrame;
    std::vector<fs::path> _ignoredPaths;
    std::vector<std::string> _ignoredChannels;
    std::map<ElementKey, std::set<fs::path>> _frameFiles;
    StaticParameters _setUpParams;
    /// number of symlinks to evaluate (0: don't follow symlinks, 1: evaluate one link, etc.)
    int _followSymlinkDepth = 1000;

    /// Read normalized depth map for frame f and stream s
    DisparityMap readNormalizedDisparityMap(StreamNumber s, FrameNumber f) const;

    /// Read depth map as stored in bag file for frame f and stream s
    DisparityMap readRawDisparityMap(StreamNumber s, FrameNumber f) const;

    /// read channel parameters (focallength, baseline, ccx, ccy)
    Eigen::MatrixXd readChannelParameters(FrameNumber f) const;

    /// read left camera picture for frame f and stream s
    Picture readPicture(StreamNumber s, FrameNumber f) const;

    /// Read rotation corrections for cameras
    std::vector<Eigen::Matrix4d> readRotationCorrections() const;

    /// Read the camera chain (8 rotation matrices that define how the cameras are positioned to each other)
    std::vector<Eigen::Matrix4d> readCamchain() const;

    /// Reads a csv file containing in each row a 4x4 matrix, stored in columnwise order
    std::vector<Eigen::Matrix4d> read4x4MatrixList(const fs::path& file) const;

    /// Reads a csv file into one matrix
    Eigen::MatrixXd readMatrix(const fs::path& file) const;

    /// constructs a filename according to the expected pattern
    std::string createFilename(const std::string& base, StreamNumber s, FrameNumber f, const std::string& suffix) const;

    /// as the other createFilename but without stream index
    std::string createFilename(const std::string& base, FrameNumber f, const std::string& suffix) const;

    /// If there are multiple files mapping to the same key, this method tries to work out the best match
    fs::path chooseCandidate(const std::set<fs::path>& candidates) const;
};


} // MouseTrack
