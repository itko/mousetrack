/// \file
/// Maintainer: Felice Serena
///
///

#include "resolve_symlink.h"

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>

namespace MouseTrack {

namespace fs = boost::filesystem;

std::string resolve_symlink(const std::string &p, int maxSymlinksFollowed) {
  fs::path path(p);
  int evaluatedSymlinks = 0;
  while (evaluatedSymlinks < maxSymlinksFollowed && fs::is_symlink(path)) {
    path = fs::read_symlink(path);
    BOOST_LOG_TRIVIAL(info) << "Followed symbolic link to: " << path.string();
    evaluatedSymlinks += 1;
  }
  return path.string();
}

} // namespace MouseTrack
