/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_timer.h"
#include <boost/log/trivial.hpp>

namespace MouseTrack {

// keys

const std::string FRAME_TOTAL{"frameTotal"};
const std::string PIPELINE_TOTAL{"pipelineTotal"};
const std::string READ_FRAME_WINDOW{"readFrameWindow"};
const std::string FRAME_WINDOW_FILTERING{"frameWindowFiltering"};
const std::string REGISTRATION{"registration"};
const std::string POINT_CLOUD_FILTERING{"pointCloudFiltering"};
const std::string CLUSTERING{"clustering"};
const std::string DESCRIPTING{"descripting"};
const std::string MATCHING{"matching"};
const std::string CONTROL_POINTS{"controlPoints"};

const std::vector<std::string> CSV_COLS{FRAME_TOTAL,
                                        READ_FRAME_WINDOW,
                                        FRAME_WINDOW_FILTERING,
                                        REGISTRATION,
                                        POINT_CLOUD_FILTERING,
                                        CLUSTERING,
                                        DESCRIPTING,
                                        MATCHING,
                                        CONTROL_POINTS};

void PipelineTimer::pipelineStarted() { startTimer(-1, PIPELINE_TOTAL); }

void PipelineTimer::pipelineTerminated() { stopTimer(-1, PIPELINE_TOTAL); }

void PipelineTimer::frameStart(FrameNumber f) { startTimer(f, FRAME_TOTAL); }

void PipelineTimer::frameEnd(FrameNumber f) {
  stopTimer(f, FRAME_TOTAL);
  auto d = _durations.find(f);
  if (_durations.end() == d) {
    BOOST_LOG_TRIVIAL(warning)
        << "No entry for frame number " << f << " found.";
  }
  // write csv row
  if (_logPath == "") {
    return;
  }
  if (!_logFileHandle.is_open()) {
    _logFileHandle.open(_logPath);
  }
  _logFileHandle << f;
  for (const auto &c : CSV_COLS) {
    _logFileHandle << ",";
    _logFileHandle << (d->second)[c];
  }
  _logFileHandle << "\n" << std::flush;
  _durations.erase(d);
}

void PipelineTimer::startFrameWindow(FrameNumber f) {
  startTimer(f, READ_FRAME_WINDOW);
}

void PipelineTimer::newFrameWindow(FrameNumber f,
                                   std::shared_ptr<const FrameWindow>) {
  stopTimer(f, READ_FRAME_WINDOW);
}

void PipelineTimer::startFrameWindowFiltering(FrameNumber f) {
  startTimer(f, FRAME_WINDOW_FILTERING);
}

void PipelineTimer::newFilteredFrameWindow(FrameNumber f,
                                           std::shared_ptr<const FrameWindow>) {
  stopTimer(f, FRAME_WINDOW_FILTERING);
}

void PipelineTimer::startRegistration(FrameNumber f) {
  startTimer(f, REGISTRATION);
}

void PipelineTimer::newRawPointCloud(FrameNumber f,
                                     std::shared_ptr<const PointCloud>) {
  stopTimer(f, REGISTRATION);
}

void PipelineTimer::startPointCloudFiltering(FrameNumber f) {
  startTimer(f, POINT_CLOUD_FILTERING);
}

void PipelineTimer::newFilteredPointCloud(FrameNumber f,
                                          std::shared_ptr<const PointCloud>) {
  stopTimer(f, POINT_CLOUD_FILTERING);
}

void PipelineTimer::startClustering(FrameNumber f) {
  startTimer(f, CLUSTERING);
}

void PipelineTimer::newClusters(FrameNumber f,
                                std::shared_ptr<const std::vector<Cluster>>) {
  stopTimer(f, CLUSTERING);
}

void PipelineTimer::startDescripting(FrameNumber f) {
  startTimer(f, DESCRIPTING);
}

void PipelineTimer::newDescriptors(
    FrameNumber f,
    std::shared_ptr<
        const std::vector<std::shared_ptr<const ClusterDescriptor>>>) {
  stopTimer(f, DESCRIPTING);
}

void PipelineTimer::startMatching(FrameNumber f) { startTimer(f, MATCHING); }

void PipelineTimer::newMatches(FrameNumber f,
                               std::shared_ptr<const std::vector<long>>) {
  stopTimer(f, MATCHING);
}
void PipelineTimer::startControlPoints(FrameNumber f) {
  startTimer(f, CONTROL_POINTS);
}
void PipelineTimer::newControlPoints(
    FrameNumber f, std::shared_ptr<const std::vector<Eigen::Vector3d>>) {
  stopTimer(f, CONTROL_POINTS);
}

void PipelineTimer::startTimer(FrameNumber f, const std::string &key) {
  _starts[Key(f, key)] = std::chrono::system_clock::now();
}

void PipelineTimer::stopTimer(FrameNumber f, const std::string &key) {
  auto start = _starts.find(Key(f, key));
  if (start == _starts.end()) {
    BOOST_LOG_TRIVIAL(warning)
        << "Couldn't find timer start for key (" << f << ", " << key << ")";
    return;
  }
  auto duration = std::chrono::system_clock::now() - start->second;
  _starts.erase(start);
  int micro =
      std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
  _durations[f][key] = micro;
  BOOST_LOG_TRIVIAL(info) << "(" << f << ", " << key << "): " << micro / 1000.0
                          << " milliseconds";
}

void PipelineTimer::logPath(const std::string &logPath) {
  _logPath = logPath;
  _logFileHandle.close();
}

const std::string &PipelineTimer::logPath() const { return _logPath; }

} // namespace MouseTrack
