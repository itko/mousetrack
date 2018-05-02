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

  _valid = true;
  // read yaml
  BOOST_LOG_TRIVIAL(debug) << "Reading camchain: " << cpath.string();
  readCamChain(camchain);

  // read bag
  BOOST_LOG_TRIVIAL(debug) << "Reading bag: " << bpath.string();
  readBag(bpath);
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
    PictureI im = readImageFromMsg(_frameMessages[f][index(s, Reference)]);
    frame.referencePicture = im.cast<PictureD::Scalar>() * (1.0 / 255);
    im = readImageFromMsg(_frameMessages[f][index(s, Disparity)]);

    frame.rawDisparityMap = im.cast<PictureD::Scalar>() * (1.0 / 255);
    frame.normalizedDisparityMap = normalizeDisparity(im);
    // camInfo holds: focallength, ccx, ccy
    const auto camInfo = _frameMessages[f][index(s, CamInfo)]
                             .instantiate<sensor_msgs::CameraInfo>();
    if (camInfo == nullptr) {
      BOOST_LOG_TRIVIAL(warning) << "Couldn't instatiate CamerInfo, setting "
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

void logIters(const std::vector<rosbag::View::iterator> &iters) {
  for (size_t i = 0; i < iters.size(); ++i) {
    BOOST_LOG_TRIVIAL(trace) << "it " << i << ": time: " << iters[i]->getTime();
  }
}

void RosBagReader::readBag(const fs::path &bagPath) {
  _bag.open(bagPath.string(), rosbag::bagmode::Read);
  // read entire bag

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

  BOOST_LOG_TRIVIAL(debug) << "Collecting bag handles.";
  // Collect handles to all relevant messages
  while (!reachedEnd(viewIters, viewEnds)) {
    logIters(viewIters);
    auto diff = largestTimeDifference(viewIters);
    if (diff > _upperFrameDuration) {
      BOOST_LOG_TRIVIAL(info) << "Frames in FrameWindow no longer sychronized, "
                                 "trying to recover. Time difference: "
                              << diff.toSec();
      // TODO
      // synchronizeViewIterators(viewIters, viewEnds);
    }
    // create all necessary MessageInstances
    std::vector<rosbag::MessageInstance> msgs;
    std::transform(viewIters.begin(), viewIters.end(), std::back_inserter(msgs),
                   [](const rosbag::View::iterator &it) { return *it; });
    _frameMessages.push_back(std::move(msgs));
    incrementIterators(viewIters);
  }
  _activeFrame = 0;
  _beginFrame = 0;
  _endFrame = _frameMessages.size();
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

void RosBagReader::synchronizeViewIterators(
    std::vector<rosbag::View::iterator> &iters,
    const std::vector<rosbag::View::iterator> &ends) const {
  // get common time
  ros::Time highestTime;
  for (const auto &v : iters) {
    highestTime = std::max(v->getTime(), highestTime);
  }

  // for each stream, find closest value to highestTime
  for (size_t i = 0; i < iters.size(); ++i) {
    auto &it = iters[i];
    auto beforeIt = it;
    int advanced = 0;
    while (it->getTime() <= highestTime && it != ends[i]) {
      beforeIt = it;
      ++it;
      ++advanced;
    }
    if (it == ends[i]) {
      BOOST_LOG_TRIVIAL(info) << "Reached end of stream, picking last iterator "
                                 "for synchronization.";
      it = beforeIt;
    }
    // we are now exactly at the largest time smaller or equal highestTime
    // check which intervall is smaller and move correspondingly
    if (it->getTime() - highestTime > highestTime - beforeIt->getTime()) {
      it = std::move(beforeIt);
      --advanced;
    }
    if (advanced > 0) {
      BOOST_LOG_TRIVIAL(trace)
          << "Advanced iterator " << i << " by " << advanced;
    }
  };
  // selfcheck
  auto diff = largestTimeDifference(iters);
  if (diff > ros::Duration(_upperFrameDuration)) {
    BOOST_LOG_TRIVIAL(warning)
        << "Couldn't synchronize stream, frames more than "
        << _upperFrameDuration << " seconds appart: " << diff;
  }
}

ros::Duration RosBagReader::largestTimeDifference(
    const std::vector<rosbag::View::iterator> &iters) const {
  ros::Time highestTime;
  for (const auto &v : iters) {
    highestTime = std::max(v->getTime(), highestTime);
  }
  ros::Time lowestTime = highestTime;
  for (const auto &v : iters) {
    lowestTime = std::min(v->getTime(), lowestTime);
  }
  assert(lowestTime <= highestTime);
  return highestTime - lowestTime;
}

void RosBagReader::incrementIterators(
    std::vector<rosbag::View::iterator> &iters) const {
  for (auto &it : iters) {
    ++it;
  }
}

bool RosBagReader::reachedEnd(
    std::vector<rosbag::View::iterator> &iters,
    const std::vector<rosbag::View::iterator> &ends) const {
  assert(iters.size() == ends.size());
  for (size_t i = 0; i < iters.size(); ++i) {
    if (iters[i] == ends[i]) {
      return true;
    }
  }
  return false;
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
