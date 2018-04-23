/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_factory.h"
#include "clustering/mean_shift.h"
#include "clustering/single_cluster.h"
#include "descripting/cog.h"
#include "matching/nearest_neighbour.h"
#include "matlab_reader.h"
#include "matlab_reader_concurrent.h"
#include "point_cloud_filtering/subsample.h"
#include "registration/disparity_registration.h"
#include "registration/disparity_registration_cpu_optimized.h"
#include "trajectory_builder/cog_trajectory_builder.h"
#include <boost/log/trivial.hpp>

namespace MouseTrack {

Pipeline
PipelineFactory::fromCliOptions(const op::variables_map &options) const {
  if (options.count("src-dir") == 0) {
    BOOST_LOG_TRIVIAL(info) << "No command line source path given.";
  }
  std::unique_ptr<Reader> reader = getReader(options);

  std::unique_ptr<Registration> registration{getRegistration(options)};

  std::unique_ptr<PointCloudFiltering> cloudFiltering{
      getCloudFiltering(options)};

  std::unique_ptr<Clustering> clustering{getClustering(options)};

  std::unique_ptr<Descripting> descripting{getDescripting(options)};

  std::unique_ptr<Matching> matching{getMatching(options)};

  std::unique_ptr<TrajectoryBuilder> trajectoryBuilder{
      getTrajectoryBuilder(options)};
  return Pipeline(std::move(reader), nullptr, std::move(registration),
                  std::move(cloudFiltering), std::move(clustering),
                  std::move(descripting), std::move(matching),
                  std::move(trajectoryBuilder));
}

std::unique_ptr<Reader>
PipelineFactory::getReader(const op::variables_map &options) const {
  std::string target = options["pipeline-reader"].as<std::string>();
  if (target == "matlab" || target == "matlab-concurrent") {
    std::unique_ptr<MatlabReader> reader;
    if (target == "matlab") {
      reader = std::unique_ptr<MatlabReader>(
          new MatlabReader(options["src-dir"].as<std::string>()));
    } else {
      reader = std::unique_ptr<MatlabReader>(
          new MatlabReaderConcurrent(options["src-dir"].as<std::string>()));
    }
    if (options.count("first-frame")) {
      reader->setBeginFrame(
          std::max(reader->beginFrame(), options["first-frame"].as<int>()));
    }
    if (options.count("last-frame")) {
      reader->setEndFrame(
          std::min(reader->endFrame(), options["last-frame"].as<int>() + 1));
    }
    if (options.count("first-stream")) {
      reader->setBeginStream(
          std::max(reader->beginStream(), options["first-stream"].as<int>()));
    }
    if (options.count("last-stream")) {
      reader->setEndStream(
          std::min(reader->endStream(), options["last-stream"].as<int>() + 1));
    }
    return reader;
  }

  return nullptr;
}

std::unique_ptr<Registration>
PipelineFactory::getRegistration(const op::variables_map &options) const {
  std::string target = options["pipeline-registration"].as<std::string>();
  if (target == "disparity") {
    return std::unique_ptr<Registration>(new DisparityRegistration());
  }
  if (target == "disparity-cpu-optimized") {
    return std::unique_ptr<Registration>(
        new DisparityRegistrationCpuOptimized());
  }
  return nullptr;
}

std::unique_ptr<PointCloudFiltering>
PipelineFactory::getCloudFiltering(const op::variables_map &options) const {
  std::string target =
      options["pipeline-point-cloud-filtering"].as<std::string>();
  if (target == "subsample") {
    int desired = options["subsample-to"].as<int>();
    return std::unique_ptr<PointCloudFiltering>(new SubSample(desired));
  }
  return nullptr;
}

std::unique_ptr<Clustering>
PipelineFactory::getClustering(const op::variables_map &options) const {
  std::string target = options["pipeline-clustering"].as<std::string>();
  if (target == "mean-shift") {
    std::unique_ptr<MeanShift> ptr{
        new MeanShift(options["mean-shift-sigma"].as<double>())};
    ptr->setMaxIterations(options["mean-shift-max-iterations"].as<int>());
    ptr->setMergeThreshold(options["mean-shift-merge-threshold"].as<double>());
    ptr->setConvergenceThreshold(
        options["mean-shift-convergence-threshold"].as<double>());
    return ptr;
} else if (target == "single-cluster") {
    std::unique_ptr<SingleCluster> ptr{new SingleCluster()};
    return ptr;
}
  return nullptr;
}

std::unique_ptr<Descripting>
PipelineFactory::getDescripting(const op::variables_map &options) const {
  std::string target = options["pipeline-descripting"].as<std::string>();
  if (target == "cog") {
    return std::unique_ptr<Descripting>(new CenterOfGravity());
  }
  return nullptr;
}

std::unique_ptr<Matching>
PipelineFactory::getMatching(const op::variables_map &options) const {
  std::string target = options["pipeline-matching"].as<std::string>();
  if (target == "nearest-neighbor") {
    return std::unique_ptr<Matching>(new NearestNeighbour());
  }
  return nullptr;
}

std::unique_ptr<TrajectoryBuilder>
PipelineFactory::getTrajectoryBuilder(const op::variables_map &options) const {
  std::string target = options["pipeline-trajectory-builder"].as<std::string>();
  if (target == "raw-cog") {
    return std::unique_ptr<TrajectoryBuilder>(new CogTrajectoryBuilder());
  }
  return nullptr;
}

} // namespace MouseTrack
