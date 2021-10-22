//------------------------------------------------------------------------------------------------------------------------------
// Copyright (c) 2021 Ana Maria Casado Fauli (anacasadofauli@gmail.com)
//------------------------------------------------------------------------------------------------------------------------------
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
// documentation files (the "Software"), to deal in the Software without restriction, including without 
// limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
// Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO 
// THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//------------------------------------------------------------------------------------------------------------------------------

#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/map_builder_interface.h>
#include <cartographer/mapping/proto/map_builder_options.pb.h>
#include <yaml-cpp/yaml.h>

using namespace cartographer;

/// This class is in charge of setting all the parameters of the MapBuilder, given the path where all the config files should exist.
class MapBuilderOptionsYaml{
    public:
        /// Constructor.
        /// \param _path path to folder of config files
        MapBuilderOptionsYaml(std::string _path);

        /// This method will return the configuration for the MapBuilder that we desire.
        mapping::proto::MapBuilderOptions getMapOpt() { return mapBuilderOpt_; }

    private:
        void setMapBuilderOptions(std::string _path);
        void setPoseGraphOptions(YAML::Node _config);

    private:
        mapping::proto::MapBuilderOptions                                       mapBuilderOpt_;
        mapping::proto::PoseGraphOptions                                        poseGraphOpt_;

        mapping::optimization::proto::OptimizationProblemOptions                optimProblemOpt_;
        common::proto::CeresSolverOptions                                       ceresSolverOptimProblemOpt_;
        mapping::constraints::proto::ConstraintBuilderOptions                   constraintOpt_;

        mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D      fastCorrScan2DOpt_;
        mapping::scan_matching::proto::CeresScanMatcherOptions2D                ceresScan2DOpt_;
        common::proto::CeresSolverOptions                                       ceresSolver2DOpt_;

        mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions3D      fastCorrScan3DOpt;
        mapping::scan_matching::proto::CeresScanMatcherOptions3D                ceresScan3DOpt_;
        common::proto::CeresSolverOptions                                       ceresSolver3DOpt_;

};

