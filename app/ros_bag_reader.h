/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "reader/reader.h"
#include "types.h"

#include <rosbag/message_instance.h>
#include <rosbag/view.h>

#include <yaml-cpp/yaml.h>

namespace MouseTrack {

/// This class reads a ROS bag in combination with a given camchain yaml.
///
/// It collects handles to all frame data in a preflight step in the constructor
/// but only reads the actual data when requested.
class RosBagReader : public Reader {
public:
  enum Channel { Reference = 0, Disparity = 1, CamInfo = 2, ChannelCount = 3 };

  RosBagReader(const fs::path &bag_path, const fs::path &camchain_path);
  RosBagReader();

  virtual bool valid() const;

  /// index of first existing frame
  virtual FrameNumber beginFrame() const;

  /// Index of first non existing frame at the end
  virtual FrameNumber endFrame() const;

  void setBeginFrame(FrameNumber begin);

  void setEndFrame(FrameNumber end);

  /// Return next frame number and increment internal pointer to next frame
  virtual FrameNumber nextFrame();

  virtual bool hasNextFrame() const;

  /// Fetch all data belonging to frame f of all streams
  virtual FrameWindow frameWindow(FrameNumber f) const;

  /// Implement interface
  virtual FrameWindow operator()(FrameNumber f);

private:
  rosbag::Bag _bag;
  /// camchain rotations as given by the yaml
  std::vector<Eigen::Matrix4d> _camchain;
  /// only rotations of reference camera, undistorted
  std::vector<Eigen::Matrix4d> _rotationCorrections;

  /// cached messageInstances for random access, according to the documentation
  /// this class is nothing more than a handle and hence lightweight
  std::vector<std::vector<rosbag::MessageInstance>> _frameMessages;

  /// Messages might be missing in the bag file, creating information holes
  /// This holds a mapping such that all entries at index `i`, e.g.
  /// `_syncedFrameMsgPtrs[..][i]`, happen at the same time
  std::vector<std::vector<size_t>> _syncedFrameMsgPtrs;

  /// mi: message index in [0, _frameMessages.size())
  ///
  /// f: index into _syncedFrameMsgPtrs
  const rosbag::MessageInstance *frameMessage(int mi, FrameNumber f) const;

  FrameNumber _activeFrame;

  bool _valid;

  int _beginFrame = 0;
  int _endFrame = 0;

  int _streamsCount = 4;

  int _followSymlinkDepth = 100;

  /// If the mesages of a frame have a larger distance than
  /// `_upperFrameDuration` seconds, we classify them as no longer synchronized
  ros::Duration _upperFrameDuration = ros::Duration(0.1);

  PictureD normalizeDisparity(const PictureI &disp) const;

  PictureI readImageFromMsg(const rosbag::MessageInstance &it) const;

  void readBag(const fs::path &bagPath);

  void readCamChain(const fs::path &camChainPath);

  Eigen::Matrix4d readYAML4x4Matrix(const YAML::Node &node) const;

  size_t index(StreamNumber s, Channel ch) const;

  Eigen::Matrix4d undistrect(Eigen::Matrix4d &R) const;
};

} // namespace MouseTrack
