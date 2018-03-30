/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "generic/types.h"
#include "generic/disparity_map.h"
#include "generic/frame_window.h"
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
    /// This is the constructor you should use, give it the root to the bag file directory
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

    /// Fetch normalized disparity map for frame f and stream s
    DisparityMap normalizedDisparityMap(StreamNumber s, FrameNumber f) const;

    /// Fetch disparity map as stored in bag file for frame f and stream s
    DisparityMap rawDisparityMap(StreamNumber s, FrameNumber f) const;

    /// Fetch channel parameters (focallength, baseline, ccx, ccy)
    Eigen::MatrixXd channelParameters(FrameNumber f) const;

    /// Fetch left camera picture for frame f and stream s
    Picture picture(StreamNumber s, FrameNumber f) const;

    /// Fetch rotation corrections for cameras
    std::vector<Eigen::Matrix4d> rotationCorrections() const;

    /// Fetch the camera chain (8 rotation matrices that define how the cameras are positioned to each other)
    std::vector<Eigen::Matrix4d> camchain() const;

    /// Fetch all data belonging to frame f of all streams
    FrameWindow frameWindow(FrameNumber f) const;
private:
    /// Collect cached values in an organized way
    struct ConstParameters {
        std::vector<Eigen::Matrix4d> rotationCorrections;
        std::vector<Eigen::Matrix4d> camchain;
    };
    /// Merge the three relevant components into one key for std::set
    struct ElementKey {
        ElementKey(StreamNumber s, FrameNumber f, std::string c) : stream(s), frame(f), channel(c) {
            // empty
        }
        StreamNumber stream;
        FrameNumber frame;
        std::string channel;
        bool operator<(const ElementKey& other) const;
    };
    /// Collect data relevant paths in an organized way
    struct FilePaths {
        std::map<ElementKey, std::set<fs::path>> frames;
        std::set<fs::path> rotationCorrectionsPath;
        std::set<fs::path> camchainPath;
    };

    /// Stores the absolute path to the root directory of data we want to read.
    fs::path _root;

    /// Set by constructor, true if there exist properly formed data
    /// false, if the constructor couldn't make sense of the given directory
    bool _valid;

    /// Holds first index for which a stream in the file system exists
    StreamNumber _beginStream;

    /// already holds C++ semantics of pointing to the first non existing entry at the end
    StreamNumber _endStream;

    /// Stores first index for which a frame in the filesyste exists
    FrameNumber _beginFrame;

    /// already holds C++ semantics of pointing to the first non existing entry at the end
    FrameNumber _endFrame;

    /// The preflight might decide to ignore paths/files it didn't expect (we don't care about
    /// additional files in the directory). Those entries are listed here.
    std::vector<fs::path> _ignoredPaths;

    /// Sometimes a file might look like a valid frame file (it has a pattern of <channelName>_s_%d_f_%d.<suffix>)
    /// But the channel name is not expected. This could be an incident or be a hit, that there's a version
    /// mismatch between MatlabReader and the matlab extraction script.
    std::vector<std::string> _ignoredChannels;

    /// We collect expected paths here
    FilePaths _files;

    /// Some files are expected to be accessed many times, we cache them here
    ConstParameters _cache;
    /// number of symlinks to evaluate (0: don't follow symlinks, 1: evaluate one link, etc.)
    int _followSymlinkDepth = 1000;

    /// Assumes an existing `_root` directory. It scans the directory and collects all filenames that might interest us.
    void preflight();

    /// Read normalized dispairty map for frame f and stream s
    DisparityMap readNormalizedDisparityMap(StreamNumber s, FrameNumber f) const;

    /// Read disparity map as stored in bag file for frame f and stream s
    DisparityMap readRawDisparityMap(StreamNumber s, FrameNumber f) const;

    /// read channel parameters: each row is a tuple of (focallength, baseline, ccx, ccy)
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

    /// If there are multiple files mapping to the same key, this method tries to work out the best match
    fs::path chooseCandidate(const std::set<fs::path>& candidates) const;
};


} // MouseTrack
