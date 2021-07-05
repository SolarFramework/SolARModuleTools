/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SOLARMODULETOOLS_TRAITS_H
#define SOLARMODULETOOLS_TRAITS_H

#include "xpcf/component/ComponentTraits.h"

namespace SolAR {
namespace MODULES {
/**
 * @namespace SolAR::MODULES::TOOLS
 * @brief <B>Provides a set of useful components</B>
 * <TT>UUID: 28b89d39-41bd-451d-b19e-d25a3d7c5797</TT>
 *
 */
namespace TOOLS {
class SolARHomographyValidation;
class SolARImage2WorldMapper4Marker2D;
class SolARSBPatternReIndexer;
class SolARKeypointsReIndexer;
class SolARKeyframeSelector;
class SolAR2DTransform;
class SolAR3DTransform;
class SolARBasicMatchesFilter;
class SolARMapFilter;
class SolARMapManager;
class SolARBasicSink;
class SolARBasicSource;
class SolARKeyframesManager;
class SolARPointCloudManager;
class SolARCovisibilityGraphManager;
class SolARBoostCovisibilityGraph;
class SolAR3D3DCorrespondencesFinder;
class SolAR3DTransformEstimationSACFrom3D3D;
class SolARLoopClosureDetector;
class SolARLoopCorrector;
class SolAROverlapDetector;
class SolARMapUpdate;
class SolARFiducialMarkerPoseEstimator;
class SolARSLAMBootstrapper;
class SolARSLAMTracking;
class SolARSLAMMapping;
class SolARStereoDepthEstimation;
class SolARStereoFeatureExtractionAndDepthEstimation;
class SolARStereoBootstrapper;
class SolARStereoReprojection;

}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolAR2DTransform,
                             "edcedc0a-9841-4377-aea1-9fa9fdb46fde",
                             "SolAR2DTransform",
                             "Applies a 2D Transform to a set of 2D points.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolAR3DTransform,
                             "f05dd955-33bd-4d52-8717-93ad298ed3e3",
                             "SolAR3DTransform",
                             "Applies a 3D Transform to a set of 3D points.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARBasicMatchesFilter,
                             "cbb620c3-a7fc-42d7-bcbf-f59b475b23b0",
                             "SolARBasicMatchesFilter",
                             "Retains the best match for each keypoint.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARBasicSink,
                             "85db2f25-4f1c-4e06-9011-e020284bfc4f",
                             "SolARBasicSink",
                             "A Sink for a synchronized pose and texture buffer based on an image buffer useful for AR video see-through pipelines.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARBasicSource,
                             "1e43cda9-7850-4a8a-a32b-f3f31ea94902",
                             "SolARBasicSource",
                             "Feeds a pipeline with an external image.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARHomographyValidation,
                             "112f9f03-79c1-4393-b8f3-e02227bebfed",
                             "SolARHomographyValidation",
                             "Checks if an homography is valid based on 4 corners of a squared marker and their projection through a given homography.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARImage2WorldMapper4Marker2D,
                             "6fed0169-4f01-4545-842a-3e2425bee248",
                             "SolARImage2WorldMapper4Marker2D",
                             "Retrieves the 3D correspondences of pixels of a 2D marker.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARKeyframeSelector,
                             "ad59a5ba-beb8-11e8-a355-529269fb1459",
                             "SolARKeyframeSelector",
                             "Defines if a frame can be a candidate for a keyframe.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARKeypointsReIndexer,
                             "c2836cc0-0344-4956-8959-84936fb4bcf2",
                             "SolARKeypointsReIndexer",
                             "Provides two ordered set of matching keypoints from two unordered set of keypoints and their corresponding matches.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARMapFilter,
                             "09205b96-7cba-4415-bc61-64744bc26222",
                             "SolARMapFilter",
                             "Filters a cloud of 3D points by removing points with a too important reporjection error or those which are behind the camera.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARMapManager,
                             "8e3c926a-0861-46f7-80b2-8abb5576692c",
                             "SolARMapManager",
                             "Updates a point map with new triangulated 3D points.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARSBPatternReIndexer,
                             "a2ef5542-029e-4fce-9974-0aea14b29d6f",
                             "SolARSBPatternReIndexer",
                             "SolAR::MODULES::TOOLS::SolARSBPatternReIndexer component")


XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARKeyframesManager,
                             "f94b4b51-b8f2-433d-b535-ebf1f54b4bf6",
                             "SolARKeyframesManager",
                             "A component to manage persistent keyframes")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARPointCloudManager,
                             "958165e9-c4ea-4146-be50-b527a9a851f0",
                             "SolARPointCloudManager",
                             "A component to manage a persistent set of 3D points")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARCovisibilityGraphManager,
                             "17c7087f-3394-4b4b-8e6d-3f8639bb00ea",
                             "SolARCovisibilityGraph",
                             "A component to manage the covisibility between keyframes")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARBoostCovisibilityGraph,
                             "b8104c93-b88a-4082-999c-802b52045043",
                             "SolARBoostCovisibilityGraph",
                             "A component to manage the covisibility between keyframes which uses the boost library")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolAR3D3DCorrespondencesFinder,
							"978068ef-7f93-41ef-8e24-13419776d9c6",
							"SolAR3D3DCorrespondencesFinder",
							"Finds the 3D-3D correspondences from feature matches of two keyframes")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolAR3DTransformEstimationSACFrom3D3D,
							"3b7a1117-8b59-46b1-8e0c-6e76a8377ab4",
							"SolAR3DTransformEstimationSACFrom3D3D",
							"Finds the 3D transform of 3D-3D points correspondences with a SAmple Consensus")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARLoopClosureDetector,
                            "e3d5946c-c1f1-11ea-b3de-0242ac130004",
                            "SolARLoopClosingDetector",
                            "Detect a loop closure from a given keyframe..")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolAROverlapDetector,
                            "58087630-1376-11eb-adc1-0242ac120002",
                            "SolAROverlapDetector",
                            "Detects overlaps between different maps.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARMapUpdate,
							"3960331a-9190-48f4-aeba-e20bf6a24465",
							"SolARMapUpdate",
							"Update the global map after merging a local map into the global map.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARLoopCorrector,
							"1007b588-c1f2-11ea-b3de-0242ac130004",
							"SolARLoopCorrector",
							"Corrects a loop of camera poses and updates associated geometry.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARFiducialMarkerPoseEstimator,
							"cddd23c4-da4e-4c5c-b3f9-7d095d097c97",
							"SolARFiducialMarkerPoseEstimator",
							"Estimate camera pose based on a fiducial marker.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARSLAMBootstrapper,
							"8f43eed0-1a2e-4c47-83f0-8dd5b259cdb0",
							"SolARSLAMBootstrapper",
							"Initialization SLAM using an image stream of a camera.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARSLAMTracking,
							"c45da19d-9637-48b6-ab52-33d3f0af6f72",
							"SolARSLAMTracking",
							"SLAM tracking.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARSLAMMapping,
							"c276bcb1-2ac8-42f2-806d-d4fe0ce7d4be",
							"SolARSLAMMapping",
							"SLAM mapping.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARStereoDepthEstimation,
							"153ffeaf-7583-44a7-bb7a-3474ba7d99cb",
							"SolARStereoDepthEstimation",
							"Depth estimation based on disparity of matched features.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARStereoFeatureExtractionAndDepthEstimation,
							"d015129a-6dff-448c-bf02-66f461ff401e",
							"SolARStereoFeatureExtractionAndDepthEstimation",
							"Perform feature extraction and keypoint depth estimation from each stereo images.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARStereoBootstrapper,
							"02064ef7-e7b9-40e2-8793-6bd177f4bc79",
                            "SolARStereoBootstrapper",
                            "Perform bootstrapper using stereo camera.")
XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARStereoReprojection,
                            "6f0c5373-1b00-41ce-ab1b-a845b83f65b3",
                            "SolARStereoReprojection",
                            "Reproject keypoints with estimating depth to 3D cloud points.")
							

#endif // SOLARMODULETOOLS_TRAITS_H


