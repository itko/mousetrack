/// \file
/// Maintainer: Felice Serena
///
///

#include "frame_window.h"

namespace MouseTrack {

FrameWindow::FrameWindow() {
  // empty
}

FrameWindow::FrameWindow(const std::vector<Frame> &frames) : _frames(frames) {
  // empty
}

FrameWindow::FrameWindow(std::vector<Frame> &&frames)
    : _frames(std::move(frames)) {
  // empty
}

std::vector<Frame> &FrameWindow::frames() { return _frames; }

const std::vector<Frame> &FrameWindow::frames() const { return _frames; }

} // namespace MouseTrack
