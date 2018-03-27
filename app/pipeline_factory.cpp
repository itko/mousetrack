/// \file
/// Maintainer: Felice Serena
///
///

#include "matlab_reader.h"
#include "pipeline_factory.h"
#include "registration/disparity_registration.h"
#include "clustering/mean_shift.h"
#include "descripting/cog.h"
#include "matching/nearest_neighbour.h"
#include <boost/log/trivial.hpp>

namespace MouseTrack {

Pipeline PipelineFactory::fromCliOptions(const op::variables_map& options) const {
    std::string src_dir;
    if(options.count("src-dir") > 0){
        src_dir = options["src-dir"].as<std::string>();
    } else {
        src_dir = "";
        BOOST_LOG_TRIVIAL(info) << "No command line source path given.";
    }
    std::unique_ptr<MatlabReader> reader{new MatlabReader(src_dir)};
    if(options.count("first-frame")) {
        reader->setBeginFrame(std::max(reader->beginFrame(), options["first-frame"].as<int>()));
    }
    if(options.count("last-frame")) {
        reader->setEndFrame(std::min(reader->endFrame(), options["last-frame"].as<int>() + 1));
    }
    std::unique_ptr<Registration> registration{new DisparityRegistration()};
    std::unique_ptr<Clustering> clustering{new MeanShift(10)};
    std::unique_ptr<Descripting> descripting{new CenterOfGravity()};
    std::unique_ptr<Matching> matching{new NearestNeighbour()};
    std::unique_ptr<TrajectoryBuilder> trajectoryBuilder;
    return Pipeline(
                std::move(reader),
                std::move(registration),
                std::move(clustering),
                std::move(descripting),
                std::move(matching),
                std::move(trajectoryBuilder)
            );
}



} // MouseTrack
