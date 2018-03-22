/// \file
/// Maintainer: Felice Serena
///
///

#include "read_csv.h"
#include "explode.h"

#include <fstream>

namespace MouseTrack {

std::vector<std::vector<std::string>> read_csv(const std::string& file, char col_delemiter, char row_delemiter){
    std::ifstream f(file, std::ios_base::in);
    if(!f.is_open()){
        throw "file could not be opened";
    }
    std::vector<std::vector<std::string>> rows;
    while(f.good()) {
        std::string line;
        std::getline(f, line, row_delemiter);
        std::vector<std::string> items = explode(line, col_delemiter);
        if(items.empty()){
            // ignore empty lines
            continue;
        }
        rows.push_back(items);
    }
    return rows;
}

} // MouseTrack
