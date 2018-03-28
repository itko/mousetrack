/// \file
/// Maintainer: Felice Serena
///
///

#pragma once


#include "frame.h"
#include <vector>

namespace MouseTrack {

/// Describes the entity that holds all frames with the same index from all streams.
class FrameWindow {
public:
    /// Construct empty frame window
    FrameWindow();

    /// Construct a new frame window with a list of frames.
    FrameWindow(const std::vector<Frame>& frames);

    /// Construct a new frame windwo with a list of frames.
    FrameWindow(std::vector<Frame>&& frames);

    /// Read-Write access to frames
    std::vector<Frame>& frames();

    /// Read access to frames
    const std::vector<Frame>& frames() const;
private:
    std::vector<Frame> _frames;
};

} // MouseTrack
