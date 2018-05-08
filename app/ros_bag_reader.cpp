/// \file
/// Maintainer: Felice Serena
///
///

#include "ros_bag_reader.h"
#include "generic/resolve_symlink.h"

#include <cstdint>
#include <iostream>

#include <image_transport/image_transport.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>

#include <Eigen/Geometry>

#include <boost/log/trivial.hpp>

namespace MouseTrack {

RosBagReader::RosBagReader(const fs::path &bag_path, const fs::path &camchain)
    : _valid(false) {

  // check bag path
  fs::path bpath = fs::absolute(bag_path);
  if (!fs::exists(bpath)) {
    BOOST_LOG_TRIVIAL(info)
        << "Path to bag " << bag_path.string()
        << " does not exist (absolute: " << bpath.string() << ".";
    throw "Bag file not found.";
  }

  bpath = resolve_symlink(bpath.string(), _followSymlinkDepth);

  if (!fs::is_regular_file(bpath)) {
    BOOST_LOG_TRIVIAL(info)
        << "Path to bag file does not point to a file: " << bpath;
    throw "Bag file path not a regular file.";
  }

  // check yaml path

  fs::path cpath = fs::absolute(camchain);
  if (!fs::exists(cpath)) {
    BOOST_LOG_TRIVIAL(info)
        << "Path to camchain yaml " << camchain.string()
        << " does not exist (absolute: " << cpath.string() << ".";
    throw "Camchain file not found.";
  }

  cpath = resolve_symlink(cpath.string(), _followSymlinkDepth);

  if (!fs::is_regular_file(cpath)) {
    BOOST_LOG_TRIVIAL(info)
        << "Path to camchain file does not point to a file: " << cpath;
    throw "Camchain file path not a regular file.";
  }

  // read yaml
  BOOST_LOG_TRIVIAL(debug) << "Reading camchain: " << cpath.string();
  readCamChain(camchain);

  // read bag
  BOOST_LOG_TRIVIAL(debug) << "Reading bag: " << bpath.string();
  readBag(bpath);

  _valid = true;
}

RosBagReader::RosBagReader() : _valid(false) {
  // empty
}

// public stuff

bool RosBagReader::valid() const { return _valid; }

FrameNumber RosBagReader::beginFrame() const { return _beginFrame; }

FrameNumber RosBagReader::endFrame() const { return _endFrame; }

FrameNumber RosBagReader::nextFrame() { return _activeFrame++; }

bool RosBagReader::hasNextFrame() const { return _activeFrame < endFrame(); }

void RosBagReader::setBeginFrame(FrameNumber f) {
  _beginFrame = std::max(f, _beginFrame);
  _activeFrame = std::max(f, _activeFrame);
}

void RosBagReader::setEndFrame(FrameNumber f) {
  _endFrame = std::min(f, _endFrame);
}

FrameWindow RosBagReader::frameWindow(FrameNumber f) const {
  FrameWindow window;
  for (StreamNumber s = 0; s < _streamsCount; ++s) {
    Frame frame;
    const rosbag::MessageInstance *msg = frameMessage(index(s, Reference), f);
    if (msg != nullptr) {
      PictureI im = readImageFromMsg(*msg);
      frame.referencePicture = im.cast<PictureD::Scalar>() * (1.0 / 255);
    }
    msg = frameMessage(index(s, Disparity), f);
    if (msg != nullptr) {
      PictureI im = readImageFromMsg(*msg);

      frame.rawDisparityMap = im.cast<PictureD::Scalar>() * (1.0 / 255);
      frame.normalizedDisparityMap = normalizeDisparity(im);
    }
    // camInfo holds: focallength, ccx, ccy
    msg = frameMessage(index(s, CamInfo), f);
    boost::shared_ptr<const sensor_msgs::CameraInfo> camInfo = nullptr;
    if (msg != nullptr) {
      camInfo = msg->instantiate<sensor_msgs::CameraInfo>();
    }
    if (camInfo == nullptr) {
      BOOST_LOG_TRIVIAL(warning) << "Couldn't instatiate CameraInfo, setting "
                                    "camera info values to zero.";
      frame.focallength = 0;
      frame.ccx = 0;
      frame.ccy = 0;
    } else {
      frame.focallength = camInfo->K[0];
      frame.ccx = camInfo->K[2];
      frame.ccy = camInfo->K[5];
    }
    // get baseline from YAML
    frame.baseline = -_camchain[2 * s + 1](0, 3);
    frame.rotationCorrection = _rotationCorrections[s];
    frame.camChainPicture = _camchain[2 * s];
    frame.camChainDisparity = _camchain[2 * s + 1];
    window.frames().push_back(std::move(frame));
  }
  return window;
}

PictureD RosBagReader::normalizeDisparity(const PictureI &disp) const {
  PictureD im(disp.rows(), disp.cols());
  // process disparity map to get a cleaned version
  // crop last 3 bits of 8 bit disparity value (contains debug info)
  for (int i = 0; i < disp.rows(); ++i) {
    for (int j = 0; j < disp.cols(); ++j) {
      // 1. first get rid of debug bits
      // 2. then scale according to fpga set up
      // adjust for extended disparity range with offset of 32 pixels and every:
      // im = disparityMap * 2 + 32;
      // 3. divide by 255 to convert from PictureI to PictureD
      im(i, j) = ((disp(i, j) >> 3) * 2 + 32) / 255.0;
    }
  }

  return im;
}

FrameWindow RosBagReader::operator()(FrameNumber f) { return frameWindow(f); }

// read stuff

void RosBagReader::readCamChain(const fs::path &yamlPath) {
  YAML::Node camchain = YAML::LoadFile(yamlPath.string());
  _camchain.resize(1);
  _camchain[0].setIdentity();
  _camchain.push_back(readYAML4x4Matrix(camchain["cam1"]["T_cn_cnm1"]));
  _camchain.push_back(readYAML4x4Matrix(camchain["cam2"]["T_cn_cnm1"]));
  _camchain.push_back(readYAML4x4Matrix(camchain["cam3"]["T_cn_cnm1"]));
  _camchain.push_back(readYAML4x4Matrix(camchain["cam4"]["T_cn_cnm1"]));
  _camchain.push_back(readYAML4x4Matrix(camchain["cam5"]["T_cn_cnm1"]));
  _camchain.push_back(readYAML4x4Matrix(camchain["cam6"]["T_cn_cnm1"]));
  _camchain.push_back(readYAML4x4Matrix(camchain["cam7"]["T_cn_cnm1"]));
  _rotationCorrections.clear();
  _rotationCorrections.push_back(undistrect(_camchain[1]));
  _rotationCorrections.push_back(undistrect(_camchain[3]));
  _rotationCorrections.push_back(undistrect(_camchain[5]));
  _rotationCorrections.push_back(undistrect(_camchain[7]));
}

Eigen::Matrix4d RosBagReader::readYAML4x4Matrix(const YAML::Node &node) const {
  Eigen::Matrix4d mat;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i, j) = node[i][j].as<double>();
    }
  }
  return mat;
}
const size_t NO_MSG = -1;

const rosbag::MessageInstance *RosBagReader::frameMessage(int mi,
                                                          FrameNumber f) const {
  auto fi = _syncedFrameMsgPtrs[mi][f];
  if (fi == NO_MSG) {
    return nullptr;
  }
  return &_frameMessages[mi][fi];
}

// handling iterators

void logIters(const std::vector<rosbag::View::iterator> &iters) {
  for (size_t i = 0; i < iters.size(); ++i) {
    BOOST_LOG_TRIVIAL(trace) << "it " << i << ": time: " << iters[i]->getTime();
  }
}

template <typename It>
bool reachedEnd(std::vector<It> &iters, const std::vector<It> &ends) {
  assert(iters.size() == ends.size());
  for (size_t i = 0; i < iters.size(); ++i) {
    if (iters[i] == ends[i]) {
      return true;
    }
  }
  return false;
}

template <typename It> void incrementIterators(std::vector<It> &iters) {
  for (auto &it : iters) {
    ++it;
  }
}

typedef std::vector<std::vector<size_t>> Ret;

// fills all vectors with empty values to make them the same length
void fillResult(Ret &result) {
  size_t max = 0;
  for (size_t i = 0; i < result.size(); ++i) {
    max = std::max(max, result[i].size());
  }
  for (size_t i = 0; i < result.size(); ++i) {
    result[i].resize(max);
    for (size_t j = result[i].size(); j < max; ++j) {
      result[i][j] = -1;
    }
  }
}

std::vector<std::vector<size_t>> unsynchronizedIterators(
    const std::vector<std::vector<rosbag::MessageInstance>> &msgs) {
  Ret result(msgs.size());
  for (size_t i = 0; i < msgs.size(); ++i) {
    result[i].resize(msgs[i].size());
    std::iota(result[i].begin(), result[i].end(), 0);
  }
  fillResult(result);
  return result;
}

// get all iterators to the messages in aligned lists
std::vector<std::vector<size_t>> synchronizedIterators(
    const std::vector<std::vector<rosbag::MessageInstance>> &msgs) {
  typedef std::pair<ros::Time, size_t> I;
  typedef std::list<I>::iterator Iter;
  typedef std::vector<Iter> IterVec;
  typedef IterVec::iterator Iter2Iter;

  /// set up

  // nothing to synchronize
  if (msgs.size() <= 1) {
    return unsynchronizedIterators(msgs);
  }

  // collect indexes to message instances, and their time values
  std::vector<std::list<I>> iters(msgs.size());
  for (size_t i = 0; i < msgs.size(); ++i) {
    for (size_t j = 0; j < msgs[i].size(); ++j) {
      iters[i].push_back(I(msgs[i][j].getTime(), j));
    }
  }

  //// insert "empty" nodes to fix synchronization of frames

  // holds a frameWindow of iterators to the time pairs
  IterVec active;
  std::transform(iters.begin(), iters.end(), std::back_inserter(active),
                 [](std::list<I> &v) { return v.begin(); });
  IterVec ends;
  std::transform(iters.begin(), iters.end(), std::back_inserter(ends),
                 [](std::list<I> &v) { return v.end(); });

  // synchronize nodes:
  // take iterator `it` with smallest time
  // check if (++it) shrinks size of time frame
  //
  // - if it does, insert empty values before every other node
  //   and increment it (this skips `it`)
  //
  // - if it doesn't, this means we found the smallest possible window
  // -> increment every iterator by one and repeat

  auto compare = [](const Iter &a, const Iter &b) {
    return a->first < b->first;
  };
  while (!reachedEnd(active, ends)) {
    BOOST_LOG_TRIVIAL(trace) << "checking:";
    std::for_each(active.begin(), active.end(), [](Iter &it) {
      BOOST_LOG_TRIVIAL(trace) << "c: " << it->first;
    });
    // clang-format off
    Iter2Iter lowestTime = std::min_element(active.begin(), active.end(), compare);
    Iter2Iter highestTime = std::max_element(active.begin(), active.end(), compare);
    ros::Duration beforeDiff = (*highestTime)->first - (*lowestTime)->first;
    // try to point to the next frame
    BOOST_LOG_TRIVIAL(trace)
        << "low: " << (*lowestTime)->first << ", hi: " << (*highestTime)->first;
    ++(*lowestTime);
    Iter2Iter newLowestTime = std::min_element(active.begin(), active.end(), compare);
    Iter2Iter newHighestTime = std::max_element(active.begin(), active.end(), compare);
    ros::Duration afterDiff = (*newHighestTime)->first - (*newLowestTime)->first;
    // clang-format on
    BOOST_LOG_TRIVIAL(trace) << "nlow: " << (*newLowestTime)->first
                             << ", nhi: " << (*newHighestTime)->first;
    // decide what to do
    if (afterDiff > beforeDiff) {
      // we would make the situation worse, so this is a local optimum
      // roll back
      --(*lowestTime);
      // increment to next frame window and leave this frame window alone
      std::for_each(active.begin(), active.end(), [](const Iter &it) {
        BOOST_LOG_TRIVIAL(trace) << "optimum: " << it->first;
      });
      std::for_each(active.begin(), active.end(), [](Iter &it) { ++it; });
      std::for_each(active.begin(), active.end(), [](const Iter &it) {
        BOOST_LOG_TRIVIAL(trace) << "to: " << it->first;
      });
    } else {
      // we need to skip --(*lowestTime), hence don't roll back lowestTime and
      // insert empty nodes before everything else to give them the same index
      for (size_t i = 0; i < active.size(); ++i) {
        auto &it = active[i];
        if (it != *lowestTime) {
          iters[i].insert(it, I(ros::Time(), -1));
        }
      }
    }
  }
  /// TODO: synchronize end

  /// Write clean result for return
  Ret result(msgs.size());
  for (size_t i = 0; i < result.size(); ++i) {
    std::transform(iters[i].begin(), iters[i].end(),
                   std::back_inserter(result[i]),
                   [](const I &pair) { return pair.second; });
  }
  // fill up end to make all vectors the same size
  fillResult(result);
  return result;
}

// actually reading the bag
void RosBagReader::readBag(const fs::path &bagPath) {
  _bag.open(bagPath.string(), rosbag::bagmode::Read);
  // read entire bag

  BOOST_LOG_TRIVIAL(debug) << "Bag opened.";
  // TODO: make this configurable regarding the number of cameras
  std::vector<std::string> queries(Channel::ChannelCount * _streamsCount);

  queries[index(0, Reference)] = "/uvc_baseboard0/cam_0/image_rect";
  queries[index(0, Disparity)] = "/uvc_baseboard0/cam_0/image_depth";
  queries[index(0, CamInfo)] = "/uvc_baseboard0/cam_0/camera_info";

  queries[index(1, Reference)] = "/uvc_baseboard0/cam_2/image_rect";
  queries[index(1, Disparity)] = "/uvc_baseboard0/cam_2/image_depth";
  queries[index(1, CamInfo)] = "/uvc_baseboard0/cam_2/camera_info";

  queries[index(2, Reference)] = "/uvc_baseboard1/cam_0/image_rect";
  queries[index(2, Disparity)] = "/uvc_baseboard1/cam_0/image_depth";
  queries[index(2, CamInfo)] = "/uvc_baseboard1/cam_0/camera_info";

  queries[index(3, Reference)] = "/uvc_baseboard1/cam_2/image_rect";
  queries[index(3, Disparity)] = "/uvc_baseboard1/cam_2/image_depth";
  queries[index(3, CamInfo)] = "/uvc_baseboard1/cam_2/camera_info";

  std::vector<rosbag::View> views(queries.size());

  for (size_t i = 0; i < queries.size(); ++i) {
    views[i].addQuery(_bag, rosbag::TopicQuery(queries[i]));
  }

  std::vector<rosbag::View::iterator> viewIters;
  std::transform(views.begin(), views.end(), std::back_inserter(viewIters),
                 [](rosbag::View &v) { return v.begin(); });

  std::vector<rosbag::View::iterator> viewEnds;
  std::transform(views.begin(), views.end(), std::back_inserter(viewEnds),
                 [](rosbag::View &v) { return v.end(); });

  _frameMessages.resize(views.size());
  BOOST_LOG_TRIVIAL(debug) << "Collecting bag handles.";
  // Collect handles to all relevant messages
  for (size_t i = 0; i < _frameMessages.size(); ++i) {
    _frameMessages[i].clear();
    while (viewIters[i] != viewEnds[i]) {
      _frameMessages[i].push_back(*viewIters[i]);
      ++viewIters[i];
    }
  }

  BOOST_LOG_TRIVIAL(debug) << "Found " << _frameMessages[0].size()
                           << " messages.";
  BOOST_LOG_TRIVIAL(debug) << "Synchronizing bag handles.";
  _syncedFrameMsgPtrs = unsynchronizedIterators(_frameMessages);

  // check framewindows
  BOOST_LOG_TRIVIAL(debug)
      << "Checking synchronization consistency of frameWindows.";
  bool sync = true;
  for (size_t i = 0; i < _syncedFrameMsgPtrs[0].size(); ++i) {
    ros::Time highest;
    for (size_t j = 0; j < _syncedFrameMsgPtrs.size(); ++j) {
      auto f = _syncedFrameMsgPtrs[j][i];
      if (f == NO_MSG) {
        continue;
      }
      rosbag::MessageInstance &msg = _frameMessages[j][f];
      highest = std::max(highest, msg.getTime());
    }
    ros::Time lowest = highest;
    for (size_t j = 0; j < _syncedFrameMsgPtrs.size(); ++j) {
      auto f = _syncedFrameMsgPtrs[j][i];
      if (f == NO_MSG) {
        continue;
      }
      rosbag::MessageInstance &msg = _frameMessages[j][f];
      lowest = std::min(lowest, msg.getTime());
    }
    ros::Duration diff = highest - lowest;
    if (diff >= _upperFrameDuration) {
      /*BOOST_LOG_TRIVIAL(warning)
          << "Frames of framewindow " << i
          << " are not synced, lowest: " << lowest << ", highest: " << highest
          << ", diff: " << diff << ", threshold: " << _upperFrameDuration;*/
      sync = false;
    }
  }

  BOOST_LOG_TRIVIAL(debug) << "Found " << _syncedFrameMsgPtrs[0].size()
                           << (sync ? " synced" : " un-synchronized")
                           << " frameWindows";

  _activeFrame = 0;
  _beginFrame = 0;
  _endFrame = _syncedFrameMsgPtrs[0].size();
}

PictureI
RosBagReader::readImageFromMsg(const rosbag::MessageInstance &it) const {
  const auto img = it.instantiate<sensor_msgs::Image>();
  if (img == nullptr) {
    BOOST_LOG_TRIVIAL(warning)
        << "Couldn't instantiate image, returning empty matrix.";
    return PictureI();
  }
  // wrap image in eigen map for easy manipulation
  Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic,
                           Eigen::RowMajor>>
      im(img->data.data(), img->height, img->width);
  PictureI mat = im.cast<PictureI::Scalar>();
  return mat;
}

// helpers

size_t RosBagReader::index(StreamNumber s, Channel ch) const {
  return Channel::ChannelCount * s + ch;
}

Eigen::Matrix4d RosBagReader::undistrect(Eigen::Matrix4d &G) const {
  // adapted from the original matlab script
  Eigen::Vector3d T = G.block<3, 1>(0, 3);
  Eigen::Matrix3d R = G.block<3, 3>(0, 0);
  Eigen::AngleAxisd om(R);
  // Bring the 2 cameras in the same orientation by rotating them "minimally":
  Eigen::Matrix3d r_r =
      Eigen::AngleAxisd(om.angle() / 2.0, -om.axis()).toRotationMatrix();
  Eigen::Matrix3d r_l = r_r.transpose();
  Eigen::Vector3d t = r_r * T;

  // Rotate both cameras so as to bring the translation vector in alignment with
  // the (1;0;0) axis:
  Eigen::Vector3d uu;
  uu.setZero();
  if (std::abs(t[0]) > std::abs(t[1])) {
    // Horizontal epipolar lines
    uu[0] = 1.0;
  } else {
    // Vertical epipolar lines end
    uu[1] = 1.0;
  }

  if (uu.dot(t) < 0) {
    uu = -uu;
  }
  Eigen::Vector3d ww = t.cross(uu);
  ww.normalize();
  ww = std::acos(std::abs(t.dot(uu)) / (t.norm() * uu.norm())) * ww;
  double angle = ww.norm();

  R = Eigen::AngleAxisd(angle, ww / angle);

  Eigen::Matrix4d Rout;
  Rout.setIdentity();
  Rout.block<3, 3>(0, 0) = R * r_l;
  return std::move(Rout);
}

} // namespace MouseTrack
