/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "pipeline.h"

namespace MouseTrack {

class Controller {
public:

    /// Called by main after creation
    virtual int main(int argc, char *argv[]) = 0;

    /// Read-write accessor
    Pipeline& pipeline();

    /// Read-only accessor
    const Pipeline& pipeline() const;
private:
    Pipeline _pipeline;
};


} // MouseTrack
