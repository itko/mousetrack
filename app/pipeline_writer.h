/// \file
/// Maintainer: Luzian Hug
///
///

#pragma once

#include "types.h"
#include "pipeline_observer.h"

namespace MouseTrack {

class PipelineWriter : public PipelineObserver {
public:
    PipelineWriter(fs::path targetDir);
    virtual void newRawPointCloud     (FrameIndex f, std::shared_ptr<const PointCloud> cloud);
    virtual void newClusters          (FrameIndex f, std::shared_ptr<const std::vector<Cluster>> clusters);
    virtual void newDescriptors       (FrameIndex f, std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>> descriptors);
    virtual void newMatches           (FrameIndex f, std::shared_ptr<const std::vector<long>> matches);
    virtual void newControlPoints     (FrameIndex f, std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints);
private:
    fs::path _outputDir;
    std::string _rawPointCloudPath;
    std::string _clustersPath;
    std::string _descriptorsPath;
    std::string _matchesPath;
    std::string _controlPointsPath;

    std::string insertFrame(const std::string& templatePath, FrameIndex f) const;
};

}
