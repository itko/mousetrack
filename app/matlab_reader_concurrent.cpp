#include "matlab_reader_concurrent.h"
#include <future>

namespace MouseTrack {

MatlabReaderConcurrent::MatlabReaderConcurrent(fs::path root_directory)
    : MatlabReader(root_directory) {
  // empty
}

FrameWindow MatlabReaderConcurrent::frameWindow(FrameNumber f) const {
  unsigned int count = endStream() - beginStream();
  std::vector<Frame> frames{count};
  Eigen::MatrixXd params = channelParameters(f);
#pragma omp parallel for
  for (int s = beginStream(); s < endStream(); s += 1) {
    auto normDispMap = std::async(std::launch::async, [this, s, f]() {
      return normalizedDisparityMap(s, f);
    });
    auto rawDispMap = std::async(
        std::launch::async, [this, s, f]() { return rawDisparityMap(s, f); });
    auto refPic = std::async(std::launch::async,
                             [this, s, f]() { return picture(s, f); });

    Frame frame;
    // fread frame-only dependent files
    frame.focallength = params(s - 1, 0);
    frame.baseline = params(s - 1, 1);
    frame.ccx = params(s - 1, 2);
    frame.ccy = params(s - 1, 3);
    // read cached data
    frame.rotationCorrection = rotationCorrections()[s - 1];
    frame.camChainPicture = camchain()[2 * (s - 1)];
    frame.camChainDisparity = camchain()[2 * (s - 1) + 1];
    // read stream dependent files
    frame.normalizedDisparityMap = normDispMap.get();
    frame.rawDisparityMap = rawDispMap.get();
    frame.referencePicture = refPic.get();
    // write back to array
    frames[s - beginStream()] = std::move(frame);
  }
  FrameWindow window;
  window.frames() = std::move(frames);
  return window;
}

} // namespace MouseTrack
