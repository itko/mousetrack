/// \file
/// Maintainer: Felice Serena
///
///

#include "explode.h"

#include<sstream>

namespace MouseTrack {

std::vector<std::string> explode(const std::string& s, char delim) {
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        result.push_back(item);
    }
    return result;
}

} // MouseTrack
