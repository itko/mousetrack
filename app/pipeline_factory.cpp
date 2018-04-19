/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_factory.h"
#include "clustering/mean_shift.h"
#include "descripting/cog.h"
#include "frame_window_filtering/disparity_bilateral.h"
#include "frame_window_filtering/disparity_gaussian_blur.h"
#include "frame_window_filtering/disparity_median.h"
#include "frame_window_filtering/disparity_morphology.h"
#include "matching/nearest_neighbour.h"
#include "matlab_reader.h"
#include "matlab_reader_concurrent.h"
#include "point_cloud_filtering/statistical_outlier_removal.h"
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

  std::unique_ptr<FrameWindowFiltering> windowFiltering =
      getWindowFiltering(options);

  std::unique_ptr<Registration> registration{getRegistration(options)};

  std::unique_ptr<PointCloudFiltering> cloudFiltering{
      getCloudFiltering(options)};

  std::unique_ptr<Clustering> clustering{getClustering(options)};

  std::unique_ptr<Descripting> descripting{getDescripting(options)};

  std::unique_ptr<Matching> matching{getMatching(options)};

  std::unique_ptr<TrajectoryBuilder> trajectoryBuilder{
      getTrajectoryBuilder(options)};
  return Pipeline(std::move(reader), std::move(windowFiltering),
                  std::move(registration), std::move(cloudFiltering),
                  std::move(clustering), std::move(descripting),
                  std::move(matching), std::move(trajectoryBuilder));
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

std::unique_ptr<FrameWindowFiltering>
PipelineFactory::getWindowFiltering(const op::variables_map &options) const {
  std::string target =
      options["pipeline-frame-window-filtering"].as<std::string>();
  if (target == "disparity-gauss") {
    auto ptr =
        std::unique_ptr<DisparityGaussianBlur>(new DisparityGaussianBlur());
    int k = options["disparity-gauss-k"].as<int>();
    double sigma = options["disparity-gauss-sigma"].as<double>();
    ptr->kx(k);
    ptr->ky(k);
    ptr->sigmax(sigma);
    ptr->sigmay(sigma);
    return ptr;
  }
  if (target == "disparity-median") {
    auto ptr = std::unique_ptr<DisparityMedian>(new DisparityMedian());
    int diameter = options["disparity-median-diameter"].as<int>();
    ptr->diameter(diameter);
    return ptr;
  }
  if (target == "disparity-bilateral") {
    auto ptr = std::unique_ptr<DisparityBilateral>(new DisparityBilateral());
    int diameter = options["disparity-bilateral-diameter"].as<int>();
    double sigmaColor = options["disparity-bilateral-sigma-color"].as<double>();
    double sigmaSpace = options["disparity-bilateral-sigma-space"].as<double>();
    ptr->diameter(diameter);
    ptr->sigmaColor(sigmaColor);
    ptr->sigmaSpace(sigmaSpace);
    return ptr;
  }
  if (target == "disparity-morphology") {
    auto ptr = std::make_unique<DisparityMorphology>();
    int diameter = options["disparity-bilateral-diameter"].as<int>();
    DisparityMorphology::KernelShape shape;
    std::string shapeStr =
        options["disparity-morphology-shape"].as<std::string>();
    if (shapeStr == "rect") {
      shape = DisparityMorphology::KernelShape::rect;
    } else if (shapeStr == "ellipse") {
      shape = DisparityMorphology::KernelShape::ellipse;
    } else if (shapeStr == "cross") {
      shape = DisparityMorphology::KernelShape::cross;
    } else {
      BOOST_LOG_TRIVIAL(warning)
          << "Unknown disparity-morphology-shape: " << shapeStr
          << ". Falling back to rect.";
      shape = DisparityMorphology::KernelShape::rect;
    }

    DisparityMorphology::Morph operation;
    std::string operationStr =
        options["disparity-morphology-operation"].as<std::string>();

    if (operationStr == "open") {
      operation = DisparityMorphology::Morph::open;
    } else if (operationStr == "close") {
      operation = DisparityMorphology::Morph::close;
    } else {
      BOOST_LOG_TRIVIAL(warning)
          << "Unknown disparity-morphology-operation: " << shapeStr
          << ". Falling back to open.";
      operation = DisparityMorphology::Morph::open;
    }

    ptr->diameter(diameter);
    ptr->kernelShape(shape);
    ptr->operation(operation);
    return ptr;
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
    return std::unique_ptr<PointCloudFiltering>(
        new SubSample(options["subsample-to"].as<int>()));
  }
  if (target == "statistical-outlier-removal") {
    double alpha = options["statistical-outlier-removal-alpha"].as<double>();
    int k = options["statistical-outlier-removal-k"].as<int>();
    return std::unique_ptr<PointCloudFiltering>(
        new StatisticalOutlierRemoval(alpha, k));
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
