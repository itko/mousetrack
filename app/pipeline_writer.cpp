/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_writer.h"
#include "generic/write_csv.h"
#include "generic/write_point_cloud.h"
#include <boost/algorithm/string/replace.hpp>
#include <boost/log/trivial.hpp>

namespace MouseTrack {

PipelineWriter::PipelineWriter(fs::path targetDir) :
    _outputDir(fs::absolute(targetDir)),
    _rawPointCloudPath("raw_point_cloud_<frameNumber>.ply"),
    _clustersPath("clusters_<frameNumber>.csv"),
    _descriptorsPath("descriptors_<frameNumber>.csv"),
    _matchesPath("matches_<frameNumber>.csv"),
    _controlPointsPath("controlPoints_<frameNumber>.csv")
{
    if(!fs::exists(_outputDir)){
        fs::create_directories(_outputDir);
    }

    if(!fs::is_directory(_outputDir)){
        BOOST_LOG_TRIVIAL(info) << "Target output path is not a directory: " << _outputDir;
        throw "Target path not a directory.";
    }
}

void PipelineWriter::newRawPointCloud     (FrameIndex f, std::shared_ptr<const PointCloud> cloud){
    fs::path path = _outputDir / insertFrame(_rawPointCloudPath, f);
    write_point_cloud(path.string(), *cloud);
}

void PipelineWriter::newClusters          (FrameIndex f, std::shared_ptr<const std::vector<Cluster>> clusters){
    std::vector<std::vector<PointIndex>> tmp;
    tmp.reserve(clusters->size());
    for(const auto& c : *clusters){
        tmp.push_back(c.points());
    }
    fs::path path = _outputDir / insertFrame(_clustersPath, f);
    write_csv(path.string(), tmp);
}

void PipelineWriter::newDescriptors       (FrameIndex f, std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>> descriptors){
    // empty
}

void PipelineWriter::newMatches           (FrameIndex f, std::shared_ptr<const std::vector<long>> matches){
    std::vector<std::vector<long>> tmp;
    tmp.push_back(*matches);

    fs::path path = _outputDir / insertFrame(_matchesPath, f);
    write_csv(path.string(), tmp);
}

void PipelineWriter::newControlPoints     (FrameIndex f, std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints){
    const int dimensions = 3;
    std::vector<std::vector<double>> tmp(controlPoints->size());
    for(size_t i = 0; i < controlPoints->size(); i += 1){
        tmp[i].resize(dimensions);
        for(int j = 0; j < dimensions; j += 1) {
            tmp[i][j] = (*controlPoints)[i][j];
        }
    }
    fs::path path = _outputDir / insertFrame(_controlPointsPath, f);
    write_csv(path.string(), tmp);
}

std::string PipelineWriter::insertFrame(const std::string& templatePath, FrameIndex f) const {
    return boost::replace_all_copy(templatePath, "<frameNumber>", std::to_string(f));
}


} // MouseTrack

