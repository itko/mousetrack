/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_writer.h"
#include "color/color.h"
#include "generic/write_csv.h"
#include "generic/write_png.h"
#include "generic/write_point_cloud.h"

#include <boost/algorithm/string/replace.hpp>
#include <boost/log/trivial.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace MouseTrack {

PipelineWriter::PipelineWriter(fs::path targetDir)
    // clang-format off
    : _outputDir(fs::absolute(targetDir)),
      _rawFrameWindowPath("raw_frame_window"),
      _rawFrameWindowDisparityPath("disparity_normalized_s_<streamNumber>_f_<frameNumber>.png"),
      _rawFrameWindowRawDisparityPath("disparity_s_<streamNumber>_f_<frameNumber>.png"),
      _rawFrameWindowReferencePath("pic_s_<streamNumber>_f_<frameNumber>.png"),
      _rawFrameWindowParamsPath("pic_f_<frameNumber>.csv"),
      _filteredFrameWindowPath("filtered_frame_window"),
      _filteredFrameWindowDisparityPath("disparity_normalized_s_<streamNumber>_f_<frameNumber>.png"),
      _filteredFrameWindowRawDisparityPath("disparity_s_<streamNumber>_f_<frameNumber>.png"),
      _filteredFrameWindowReferencePath("pic_s_<streamNumber>_f_<frameNumber>.png"),
      _filteredFrameWindowParamsPath("pic_f_<frameNumber>.csv"),
      _filteredFrameWindowLabelsPath("label_s_<streamNumber>_l_<labelNumber>_f_<frameNumber>.png"),
      _rawPointCloudPath("raw_point_cloud_<frameNumber>.ply"),
      _rawPointCloudMetricsPath("raw_point_cloud_metrics_<frameNumber>.csv"),
      _filteredPointCloudPath("filtered_point_cloud_<frameNumber>.ply"),
      _filteredPointCloudMetricsPath("filtered_point_cloud_metrics_<frameNumber>.txt"),
      _clusteredPointCloudPath("clustered_point_cloud_<frameNumber>.ply"),
      _clustersPath("clusters_<frameNumber>.csv"),
      _clustersCoGsPath("cluster_cogs_<frameNumber>.csv"),
      _descriptorsPath("descriptors_<frameNumber>.csv"),
      _matchesPath("matches_<frameNumber>.csv"),
      _controlPointsPath("controlPoints_<frameNumber>.csv"),
      _chainedPointCloudPath("chained_point_cloud_<frameNumber>.ply")
// clang-format on
{
  if (!fs::exists(_outputDir)) {
    fs::create_directories(_outputDir);
  }

  if (!fs::is_directory(_outputDir)) {
    BOOST_LOG_TRIVIAL(info)
        << "Target output path is not a directory: " << _outputDir;
    throw "Target path not a directory.";
  }
}

void PipelineWriter::newFrameWindow(FrameNumber f,
                                    std::shared_ptr<const FrameWindow> window) {
  if (!writeRawFrameWindow) {
    return;
  }
  fs::path base = _outputDir / insertFrame(_rawFrameWindowPath, f);
  if (!fs::exists(base)) {
    fs::create_directory(base);
  }
  Eigen::MatrixXd params(window->frames().size(), 4);
  for (StreamNumber s = 0; (size_t)s < window->frames().size(); ++s) {
    // clang-format off
    fs::path ref = base / insertFrame(insertStream(_rawFrameWindowReferencePath, s), f);
    fs::path dispNorm = base / insertFrame(insertStream(_rawFrameWindowRawDisparityPath, s), f);
    fs::path dispRaw = base / insertFrame(insertStream(_rawFrameWindowDisparityPath, s), f);
    // clang-format on
    const Frame &frame = window->frames()[s];

    writePng(frame.referencePicture, ref.string());
    writePng(frame.rawDisparityMap, dispRaw.string());
    writePng(frame.normalizedDisparityMap, dispNorm.string());

    params(s, 0) = frame.focallength;
    params(s, 1) = frame.baseline;
    params(s, 2) = frame.ccx;
    params(s, 3) = frame.ccy;
  }
  fs::path paramsPath = base / insertFrame(_rawFrameWindowParamsPath, f);
  std::vector<std::vector<Precision>> paramsVec(window->frames().size());
  const int paramCount = 4;
  for (int i = 0; (size_t)i < window->frames().size(); ++i) {
    for (int j = 0; j < paramCount; ++j) {
      paramsVec[i].push_back(params(i, j));
    }
  }
  write_csv(paramsPath.string(), paramsVec);
}

void PipelineWriter::newFilteredFrameWindow(
    FrameNumber f, std::shared_ptr<const FrameWindow> window) {
  if (!writeFilteredFrameWindow) {
    return;
  }
  fs::path base = _outputDir / insertFrame(_filteredFrameWindowPath, f);
  if (!fs::exists(base)) {
    fs::create_directory(base);
  }
  Eigen::MatrixXd params(window->frames().size(), 4);
  for (StreamNumber s = 0; (size_t)s < window->frames().size(); ++s) {
    // clang-format off
    fs::path ref = base / insertFrame(insertStream(_filteredFrameWindowReferencePath, s), f);
    fs::path dispNorm = base / insertFrame(insertStream(_filteredFrameWindowRawDisparityPath, s), f);
    fs::path dispRaw = base / insertFrame(insertStream(_filteredFrameWindowDisparityPath, s), f);
    // clang-format on
    const Frame &frame = window->frames()[s];

    writePng(frame.referencePicture, ref.string());
    writePng(frame.rawDisparityMap, dispRaw.string());
    writePng(frame.normalizedDisparityMap, dispNorm.string());

    for (size_t l = 0; l < frame.labels.size(); ++l) {
      const auto &label = frame.labels[l];
      fs::path labelP =
          base /
          insertLabel(
              insertFrame(insertStream(_filteredFrameWindowLabelsPath, s), f),
              l);
      writePng(label, labelP.string());
    }
    // write max-label image
    if (!frame.labels.empty()) {
      int rows = frame.labels[0].rows();
      int cols = frame.labels[0].cols();
      auto colors = nColors(frame.labels.size());
      colors[0] = std::vector<double>{255, 0, 0};
      colors[1] = std::vector<double>{0, 255, 0};
      colors[2] = std::vector<double>{0, 0, 255};
      colors[3] = std::vector<double>{125, 0, 0};
      colors[4] = std::vector<double>{0, 125, 0};
      // colors[5] = std::vector<double>{0, 0, 125};

      // ignore background
      std::set<size_t> ignore{5};
      cv::Mat allLabels(rows, cols, CV_64FC3);
      for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
          double v = 0;
          int maxL = 0;
          for (int l = 0; l < frame.labels.size(); ++l) {
            if (ignore.find(l) != ignore.end()) {
              continue;
            }
            auto c = frame.labels[l](y, x);
            if (v < c) {
              v = c;
              maxL = l;
            }
          }
          allLabels.at<cv::Vec3d>(y, x)[0] = colors[maxL][0];
          allLabels.at<cv::Vec3d>(y, x)[1] = colors[maxL][1];
          allLabels.at<cv::Vec3d>(y, x)[2] = colors[maxL][2];
        }
      }
      fs::path allPath =
          base /
          insertLabel(
              insertFrame(insertStream(_filteredFrameWindowLabelsPath, s), f),
              9999);

      try {
        cv::imwrite(allPath.string(), allLabels);
      } catch (std::runtime_error &ex) {
        BOOST_LOG_TRIVIAL(warning)
            << "Exception converting image to PNG format: " << ex.what();
      }
    }

    params(s, 0) = frame.focallength;
    params(s, 1) = frame.baseline;
    params(s, 2) = frame.ccx;
    params(s, 3) = frame.ccy;
  }
  fs::path paramsPath = base / insertFrame(_filteredFrameWindowParamsPath, f);
  std::vector<std::vector<Precision>> paramsVec(window->frames().size());
  const int paramCount = 4;
  for (int i = 0; (size_t)i < window->frames().size(); ++i) {
    for (int j = 0; j < paramCount; ++j) {
      paramsVec[i].push_back(params(i, j));
    }
  }
  write_csv(paramsPath.string(), paramsVec);
}

void PipelineWriter::newRawPointCloud(FrameNumber f,
                                      std::shared_ptr<const PointCloud> cloud) {
  _clouds[f] = cloud;
  if (!writeRawPointCloud) {
    return;
  }
  fs::path path = _outputDir / insertFrame(_rawPointCloudPath, f);
  fs::path pathMetrics = _outputDir / insertFrame(_rawPointCloudMetricsPath, f);
  write_point_cloud(path.string(), *cloud);
  write_point_cloud_metrics(pathMetrics.string(), *cloud);
}

void PipelineWriter::newFilteredPointCloud(
    FrameNumber f, std::shared_ptr<const PointCloud> cloud) {
  // overwrite rawPointCloud
  _clouds[f] = cloud;
  if (!writeFilteredPointCloud) {
    return;
  }

  fs::path path = _outputDir / insertFrame(_filteredPointCloudPath, f);
  fs::path pathMetrics =
      _outputDir / insertFrame(_filteredPointCloudMetricsPath, f);
  write_point_cloud(path.string(), *cloud);
  write_point_cloud_metrics(pathMetrics.string(), *cloud);
}

void PipelineWriter::newClusters(
    FrameNumber f, std::shared_ptr<const std::vector<Cluster>> clusters) {
  _clusters[f] = clusters;
  if (!writeClusters) {
    return;
  }
  std::vector<std::vector<PointIndex>> tmp;
  tmp.reserve(clusters->size());
  for (const auto &c : *clusters) {
    tmp.push_back(c.points());
  }
  fs::path path = _outputDir / insertFrame(_clustersPath, f);
  write_csv(path.string(), tmp);

  // write clustered point cloud
  PointCloud cloud = *_clouds[f];
  BOOST_LOG_TRIVIAL(trace) << "Writing point cloud with " << cloud.size()
                           << " points.";
  std::vector<Cluster> largeClusters;
  for (auto &c : *clusters) {
    if (c.points().size() < 10) {
      continue;
    }
    largeClusters.push_back(c);
  }
  auto clusterColors = nColors(largeClusters.size());
  for (size_t ci = 0; ci < largeClusters.size(); ++ci) {
    const auto &cluster = largeClusters[ci];
    for (auto i : cluster.points()) {
      assert(i < cloud.size());
      auto color = clusterColors[ci];
      cloud[i].r(color[0]);
      cloud[i].g(color[1]);
      cloud[i].b(color[2]);
    }
  }
  fs::path cloudPath = _outputDir / insertFrame(_clusteredPointCloudPath, f);
  write_point_cloud(cloudPath.string(), cloud);

  fs::path cogsPath = _outputDir / insertFrame(_clustersCoGsPath, f);
  std::vector<std::vector<double>> controlPoints;
  for (const auto &c : *clusters) {
    const auto p = c.center_of_gravity(cloud);
    controlPoints.push_back(std::vector<double>{p[0], p[1], p[2]});
  }

  write_csv(cogsPath.string(), controlPoints);
}

void PipelineWriter::newDescriptors(
    FrameNumber,
    std::shared_ptr<
        const std::vector<std::shared_ptr<const ClusterDescriptor>>>) {
  if (!writeDescriptors) {
    return;
  }
  // empty
}

void PipelineWriter::newMatches(
    FrameNumber f, std::shared_ptr<const std::vector<long>> matches) {
  if (!writeMatches) {
    return;
  }

  std::vector<std::vector<long>> tmp;
  tmp.push_back(*matches);

  fs::path path = _outputDir / insertFrame(_matchesPath, f);
  write_csv(path.string(), tmp);
}

void PipelineWriter::newControlPoints(
    FrameNumber f,
    std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints) {
  if (!writeControlPoints) {
    return;
  }

  const int dimensions = 3;
  std::vector<std::vector<double>> tmp(controlPoints->size());
  for (size_t i = 0; i < controlPoints->size(); i += 1) {
    tmp[i].resize(dimensions);
    for (int j = 0; j < dimensions; j += 1) {
      tmp[i][j] = (*controlPoints)[i][j];
    }
  }
  fs::path path = _outputDir / insertFrame(_controlPointsPath, f);
  write_csv(path.string(), tmp);
}

std::string PipelineWriter::insertLabel(const std::string &templatePath,
                                        int l) const {
  return boost::replace_all_copy(templatePath, "<labelNumber>",
                                 std::to_string(l));
}

std::string PipelineWriter::insertFrame(const std::string &templatePath,
                                        FrameNumber f) const {
  return boost::replace_all_copy(templatePath, "<frameNumber>",
                                 std::to_string(f));
}

std::string PipelineWriter::insertStream(const std::string &templatePath,
                                         StreamNumber s) const {
  return boost::replace_all_copy(templatePath, "<streamNumber>",
                                 std::to_string(s));
}

void PipelineWriter::newClusterChains(
    std::shared_ptr<const std::vector<ClusterChain>> chains) {
  if (!writeClusteredPointCloud) {
    return;
  }
  auto clusterColors = nColors(chains->size());
  for (auto cloudIt : _clouds) {
    FrameNumber f = cloudIt.first;
    PointCloud cloud = *cloudIt.second;
    for (size_t chainIndex = 0; chainIndex < chains->size(); ++chainIndex) {
      const ClusterChain &chain = (*chains)[chainIndex];
      const auto &clus = chain.clusters();
      auto cluster = clus.find(f);
      auto end = clus.end();
      if (cluster == end) {
        continue;
      }
      const Cluster &clu = *(cluster->second);
      for (auto i : clu.points()) {
        assert(i < cloud.size());
        auto color = clusterColors[chainIndex];
        cloud[i].r(color[0]);
        cloud[i].g(color[1]);
        cloud[i].b(color[2]);
      }
    }
    fs::path cloudPath = _outputDir / insertFrame(_chainedPointCloudPath, f);
    write_point_cloud(cloudPath.string(), cloud);
  }
}

std::vector<std::vector<double>> PipelineWriter::nColors(int n) const {
  auto cols = GenerateNColors(n);
  return cols;
}

void PipelineWriter::writePng(const PictureD &pic,
                              const std::string &path) const {
  if (pic.size() == 0) {
    return;
  }
  write_png(pic, path);
}

} // namespace MouseTrack
