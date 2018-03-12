/// \file
/// Maintainer: Felice Serena
///
/// This file is an example for a library file.
///

#pragma once // new fancy header guard

#include<vector>

/// The Mousetrack namespace holds all declarations available from the library.
namespace Mousetrack {

/// Add two STL vectors a and b
std::vector<double> add(const std::vector<double>& a, const std::vector<double>& b);

}
